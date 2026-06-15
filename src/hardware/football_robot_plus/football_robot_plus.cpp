#include "hardware/robot_selector.h"

#include "chassis/x_chassis.h"
#include "chassis/x_drive.h"
#include "hardware/football_robot_plus/external_vision_serial.h"
#include "hardware/football_robot_plus/robot_hardware.h"
#include "hardware/football_robot_plus/robot_state.h"
#include "hardware/football_robot_plus/vision.h"
#include "input/controller.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace basic::hardware::football_robot_plus {

namespace {

inline constexpr int kBackgroundLoopDelayMs = kRefreshTime;
inline constexpr double kEstimatedPositionYOffsetMm = 30.0;
inline constexpr int kVisionStaleTimeoutMs = 500;
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kAutoHeadingToleranceRad = 0.09;
inline constexpr double kAutoPickupHeadingToleranceRad = 0.05;
inline constexpr double kAutoTargetRangeMm = 220.0;
inline constexpr double kAutoPickupRangeMm = 180.0;
inline constexpr double kAutoForwardGainPctPerMm = 0.04;
inline constexpr double kAutoTurnGainPctPerRad = 180.0;
inline constexpr double kAutoMinTurnPct = 8.0;
inline constexpr double kAutoMaxForwardPct = 25.0;
inline constexpr double kAutoTurningForwardLimitPct = 10.0;
inline constexpr double kAutoMaxTurnPct = 60.0;
inline constexpr double kAutoTurnDirectionSign = -1.0;
inline constexpr double kInterceptLineHalfWidthMm = 900.0;
inline constexpr double kInterceptPose45Deg = 45.0;
inline constexpr double kInterceptPoseToleranceDeg = 4.0;
inline constexpr double kInterceptPoseTurnGainPctPerDeg = 1.0;
inline constexpr double kInterceptPoseTurnMinPct = 12.0;
inline constexpr double kInterceptPoseTurnMaxPct = 40.0;
inline constexpr double kInterceptLineGainPctPerMm = 0.30;
inline constexpr double kInterceptLineMinPct = 22.0;
inline constexpr double kInterceptLineMaxPct = 100.0;
inline constexpr double kInterceptLineAlignedToleranceMm = 35.0;
inline constexpr int kInterceptScanHoldMs = 350;
inline constexpr int kInterceptPoseHoldMs = 220;
inline constexpr double kInterceptPoseSwitchHysteresisMm = 70.0;
inline constexpr double kInterceptMaxForwardJumpMm = 500.0;
inline constexpr double kInterceptMaxLateralJumpMm = 700.0;
inline constexpr int kBallTrackHistorySize = 8;
inline constexpr int kBallTrackMinSamples = 4;
inline constexpr int kBallTrackHistoryMaxAgeMs = 800;
inline constexpr int kBallTrackMinDeltaMs = 25;
inline constexpr int kExternalVisionStaleTimeoutMs = 500;
inline constexpr int kPoseReadoutHoldMs = 1500;

enum class AutoMode {
  kManual,
  kFaceTarget,
  kIntercept,
};

enum class InterceptPose {
  kForward,
  kLeft45,
  kRight45,
};

struct BallTrackSample {
  double x_mm{0.0};
  double z_mm{0.0};
  int time_ms{0};
  bool valid{false};
};

struct InterceptPrediction {
  bool valid{false};
  bool has_ball{false};
  bool in_bounds{false};
  double current_forward_mm{0.0};
  double current_right_mm{0.0};
  double intercept_right_mm{0.0};
  double heading_error_rad{basic::vision::nan_value()};
};

struct InterceptState {
  std::array<BallTrackSample, kBallTrackHistorySize> history{};
  int history_count{0};
  int last_seen_time_ms{0};
  int scan_phase{0};
  int scan_phase_started_ms{0};
  double line_heading_deg{0.0};
  double world_s_robot_mm{0.0};
  double last_odom_forward_mm{0.0};
  double last_odom_strafe_mm{0.0};
  bool pose_initialized{false};
  InterceptPose target_pose{InterceptPose::kForward};
  int target_pose_started_ms{0};
  int last_pose_switch_ms{0};
};

double clamp_value(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

double clamp_abs(double value, double max_abs) {
  return clamp_value(value, -max_abs, max_abs);
}

bool is_positive_finite(double value) {
  return basic::vision::is_finite(value) && value > 0.0;
}

bool is_finite(double value) {
  return basic::vision::is_finite(value);
}

double radians_to_degrees(double radians) {
  return radians * 180.0 / kPi;
}

double degrees_to_radians(double degrees) {
  return degrees * kPi / 180.0;
}

double normalize_angle_deg(double angle_deg) {
  while (angle_deg > 180.0) {
    angle_deg -= 360.0;
  }
  while (angle_deg <= -180.0) {
    angle_deg += 360.0;
  }
  return angle_deg;
}

double signed_min_speed(double value, double min_abs) {
  if (std::fabs(value) < min_abs) {
    return value >= 0.0 ? min_abs : -min_abs;
  }
  return value;
}

bool has_fresh_external_detection(const FootballVisionState& vision, int now_ms) {
  if (!vision.external_link.online || !vision.last_detection.has_detection) {
    return false;
  }
  if (now_ms < vision.last_detection_rx_time_ms) {
    return false;
  }
  return now_ms - vision.last_detection_rx_time_ms <= kExternalVisionStaleTimeoutMs;
}

const char* target_color_name(basic::identify::VisionTargetColor color) {
  switch (color) {
    case basic::identify::VisionTargetColor::kRed:
      return "RED";
    case basic::identify::VisionTargetColor::kYellowGreen:
      return "YLWGRN";
    case basic::identify::VisionTargetColor::kPurple:
      return "PURPLE";
    default:
      return "UNKNOWN";
  }
}

FootballVisionConfig default_vision_config_for_sensor() {
  FootballVisionConfig config;
  config.image_width_px = 640.0;
  config.image_height_px = 480.0;
  config.camera.fx = 660.037869;
  config.camera.fy = 932.486763;
  config.camera.cx = 332.090692;
  config.camera.cy = 91.419530;
  config.camera.k1 = 0.392595721;
  config.camera.k2 = -0.282416438;
  config.camera.p1 = -0.128963182;
  config.camera.p2 = 0.005571122;
  config.camera.k3 = 0.517875520;
  config.football_diameter_mm = kDefaultFootballDiameterMm;
  config.expected_class_id = -1;
  config.camera_extrinsics.x_mm = -50.0;
  config.camera_extrinsics.y_mm = 0.0;
  config.camera_extrinsics.z_mm = 150.0;
  config.camera_extrinsics.roll_deg = 0.0;
  config.camera_extrinsics.pitch_deg = 0.0;
  // Camera is mounted upside down, so rotate its image axes 180 deg about
  // the optical axis before interpreting detections in robot coordinates.
  config.camera_extrinsics.yaw_deg = 180.0;
  return config;
}

class FootballRobotPlus;
FootballRobotPlus& current_football_robot_plus();

class FootballRobotPlus final : public basic::app::Robot {
 public:
  void initialize() override {
    configure_vision(default_vision_config_for_sensor());
    hardware_.calibrate_inertial_sensor();
    hardware_.inertial.resetRotation();
    hardware_.inertial.resetHeading();
    basic::chassis::x_chassis_reset_odometry(hardware_.football_chassis, 0.0, 0.0);
    state_.autonomous = basic::hardware::shared::AutonomousState{};
    state_.autonomous.initialized = true;
    state_.autonomous.target_heading_deg = 0.0;
    state_.autonomous.estimated_heading_deg = 0.0;
    state_.autonomous.estimated_x_mm = 0.0;
    state_.autonomous.estimated_y_mm = 0.0;
    hardware_.external_vision.initialize();
    set_vision_target_color(basic::identify::VisionTargetColor::kRed);
    hardware_.show_calibrated();
    show_mode_status();
  }

  void bind_background_tasks() override {
    vex::thread background(start_background_tasks);
  }

  void bind_competition(vex::competition& competition) override {
    competition_ = &competition;
    competition.autonomous(start_autonomous_entry);
    competition.drivercontrol(start_driver_control_entry);
  }

  void configure_vision(const FootballVisionConfig& config) {
    const basic::identify::VisionTargetColor target_color = state_.vision.target_color;
    const basic::identify::LargestBlobDetection last_blob_detection =
        state_.vision.last_blob_detection;
    const YoloDetection last_detection = state_.vision.last_detection;
    const basic::vision::EstimateResult last_estimate = state_.vision.last_estimate;
    const bool estimate_available = state_.vision.estimate_available;
    const bool class_filter_passed = state_.vision.class_filter_passed;
    const int last_detection_rx_time_ms = state_.vision.last_detection_rx_time_ms;
    const ExternalVisionLinkState external_link = state_.vision.external_link;
    state_.vision = FootballVisionState{};
    state_.vision.config = config;
    state_.vision.target_color = target_color;
    state_.vision.last_blob_detection = last_blob_detection;
    state_.vision.last_blob_detection.color = target_color;
    state_.vision.last_detection = last_detection;
    state_.vision.last_estimate = last_estimate;
    state_.vision.estimate_available = estimate_available;
    state_.vision.class_filter_passed = class_filter_passed;
    state_.vision.last_detection_rx_time_ms = last_detection_rx_time_ms;
    state_.vision.external_link = external_link;
    locator_.set_config(config.estimator);
    state_.vision.last_update_time_ms = hardware_.brain.timer(vex::timeUnits::msec);
  }

  void set_vision_target_color(basic::identify::VisionTargetColor color) {
    state_.vision.target_color = color;
    state_.vision.last_blob_detection.color = color;
    state_.vision.external_link.reported_color_code = basic::identify::color_code(color);
  }

  basic::identify::VisionTargetColor vision_target_color() const {
    return state_.vision.target_color;
  }

  basic::identify::LargestBlobDetection vision_sensor_detection() const {
    return state_.vision.last_blob_detection;
  }

  basic::vision::EstimateResult submit_yolo_detection(const YoloDetection& detection) {
    state_.vision.last_blob_detection.sensor_installed = state_.vision.external_link.online;
    state_.vision.last_blob_detection.has_detection = detection.has_detection;
    state_.vision.last_blob_detection.signature_id = detection.class_id;
    state_.vision.last_blob_detection.object_count = detection.has_detection ? 1 : 0;
    state_.vision.last_blob_detection.origin_x_px = static_cast<int>(detection.bbox_px.x);
    state_.vision.last_blob_detection.origin_y_px = static_cast<int>(detection.bbox_px.y);
    state_.vision.last_blob_detection.center_x_px =
        static_cast<int>(detection.bbox_px.x + detection.bbox_px.width * 0.5);
    state_.vision.last_blob_detection.center_y_px =
        static_cast<int>(detection.bbox_px.y + detection.bbox_px.height * 0.5);
    state_.vision.last_blob_detection.width_px = static_cast<int>(detection.bbox_px.width);
    state_.vision.last_blob_detection.height_px = static_cast<int>(detection.bbox_px.height);
    state_.vision.last_blob_detection.image_width_px = detection.image_width_px;
    state_.vision.last_blob_detection.image_height_px = detection.image_height_px;
    state_.vision.last_detection = detection;
    state_.vision.class_filter_passed =
        !detection.has_detection || state_.vision.config.expected_class_id < 0 ||
        detection.class_id == state_.vision.config.expected_class_id;
    state_.vision.last_update_time_ms = hardware_.brain.timer(vex::timeUnits::msec);
    state_.vision.last_detection_rx_time_ms = state_.vision.last_update_time_ms;

    if (!detection.has_detection) {
      state_.vision.last_estimate = basic::vision::EstimateResult{};
      state_.vision.estimate_available = false;
      return state_.vision.last_estimate;
    }

    state_.vision.last_estimate =
        basic::hardware::football_robot_plus::estimate_football_from_yolo(
            locator_, state_.vision.config, detection);
    if (state_.vision.last_estimate.valid) {
      state_.vision.last_estimate.position_camera_mm.y -= kEstimatedPositionYOffsetMm;
    }
    state_.vision.estimate_available = true;
    return state_.vision.last_estimate;
  }

  void clear_yolo_detection() {
    state_.vision.last_blob_detection.has_detection = false;
    state_.vision.last_blob_detection.object_count = 0;
    state_.vision.last_blob_detection.signature_id = 0;
    state_.vision.last_blob_detection.origin_x_px = 0;
    state_.vision.last_blob_detection.origin_y_px = 0;
    state_.vision.last_blob_detection.center_x_px = 0;
    state_.vision.last_blob_detection.center_y_px = 0;
    state_.vision.last_blob_detection.width_px = 0;
    state_.vision.last_blob_detection.height_px = 0;
    state_.vision.last_detection = YoloDetection{};
    state_.vision.last_estimate = basic::vision::EstimateResult{};
    state_.vision.estimate_available = false;
    state_.vision.class_filter_passed = true;
    state_.vision.last_update_time_ms = hardware_.brain.timer(vex::timeUnits::msec);
  }

  FootballVisionState vision_state() const { return state_.vision; }

 private:
  double local_inertial_heading_deg() {
    return normalize_angle_deg(hardware_.inertial.rotation(vex::deg));
  }

  basic::chassis::XChassisOdometry& odometry() {
    return basic::chassis::x_chassis_odometry(hardware_.football_chassis);
  }

  void reset_intercept_state() {
    intercept_state_ = InterceptState{};
    intercept_state_.line_heading_deg = local_inertial_heading_deg();
    intercept_state_.world_s_robot_mm = 0.0;
    intercept_state_.last_odom_forward_mm = odometry().x_m * 1000.0;
    intercept_state_.last_odom_strafe_mm = odometry().y_m * 1000.0;
    intercept_state_.pose_initialized = true;
    intercept_state_.target_pose = InterceptPose::kForward;
    intercept_state_.scan_phase_started_ms = hardware_.brain.timer(vex::timeUnits::msec);
    intercept_state_.target_pose_started_ms = intercept_state_.scan_phase_started_ms;
    intercept_state_.last_pose_switch_ms = intercept_state_.scan_phase_started_ms;
  }

  void request_intercept_pose(InterceptPose pose, int now_ms) {
    if (pose == intercept_state_.target_pose) {
      return;
    }
    if (now_ms - intercept_state_.last_pose_switch_ms < kInterceptPoseHoldMs) {
      return;
    }
    intercept_state_.target_pose = pose;
    intercept_state_.target_pose_started_ms = now_ms;
    intercept_state_.last_pose_switch_ms = now_ms;
  }

  bool has_intercept_pose(InterceptPose pose) {
    const double error_deg =
        std::fabs(normalize_angle_deg(local_inertial_heading_deg() - intercept_pose_heading_deg(pose)));
    return error_deg <= kInterceptPoseToleranceDeg;
  }

  double intercept_pose_heading_deg(InterceptPose pose) const {
    switch (pose) {
      case InterceptPose::kLeft45:
        return normalize_angle_deg(intercept_state_.line_heading_deg - kInterceptPose45Deg);
      case InterceptPose::kRight45:
        return normalize_angle_deg(intercept_state_.line_heading_deg + kInterceptPose45Deg);
      case InterceptPose::kForward:
      default:
        return intercept_state_.line_heading_deg;
    }
  }

  bool resolve_ball_position_in_start_frame(
      const FootballVisionState& vision,
      double* line_normal_mm,
      double* line_position_mm) {
    if (line_normal_mm == nullptr || line_position_mm == nullptr ||
        !vision.estimate_available || !vision.last_estimate.valid) {
      return false;
    }

    const basic::vision::Vec3 robot_position = camera_point_to_robot_frame(
        vision.config.camera_extrinsics,
        vision.last_estimate.position_camera_mm);
    if (!is_finite(robot_position.x) || !is_finite(robot_position.z)) {
      return false;
    }

    const double heading_error_rad = degrees_to_radians(
        normalize_angle_deg(local_inertial_heading_deg() - intercept_state_.line_heading_deg));
    const double cos_robot = std::cos(heading_error_rad);
    const double sin_robot = std::sin(heading_error_rad);

    const double body_forward_mm = robot_position.z;
    const double body_right_mm = robot_position.x;
    const double ball_line_normal_mm =
        body_forward_mm * cos_robot - body_right_mm * sin_robot;
    const double ball_line_position_mm =
        intercept_state_.world_s_robot_mm +
        body_forward_mm * sin_robot + body_right_mm * cos_robot;

    *line_normal_mm = ball_line_normal_mm;
    *line_position_mm = ball_line_position_mm;
    return true;
  }

  void push_ball_track_sample(double line_normal_mm, double line_position_mm, int now_ms) {
    if (intercept_state_.history_count > 0) {
      const BallTrackSample& last =
          intercept_state_.history[intercept_state_.history_count - 1];
      if (last.valid) {
        const int dt_ms = now_ms - last.time_ms;
        if (dt_ms < kBallTrackMinDeltaMs) {
          return;
        }
        if (std::fabs(line_normal_mm - last.x_mm) > kInterceptMaxForwardJumpMm ||
            std::fabs(line_position_mm - last.z_mm) > kInterceptMaxLateralJumpMm) {
          return;
        }
      }
    }

    if (intercept_state_.history_count < kBallTrackHistorySize) {
      intercept_state_.history[intercept_state_.history_count++] =
          BallTrackSample{line_normal_mm, line_position_mm, now_ms, true};
      return;
    }

    for (int i = 1; i < kBallTrackHistorySize; ++i) {
      intercept_state_.history[i - 1] = intercept_state_.history[i];
    }
    intercept_state_.history[kBallTrackHistorySize - 1] =
        BallTrackSample{line_normal_mm, line_position_mm, now_ms, true};
  }

  void prune_ball_track_history(int now_ms) {
    int write_index = 0;
    for (int i = 0; i < intercept_state_.history_count; ++i) {
      const BallTrackSample sample = intercept_state_.history[i];
      if (!sample.valid) {
        continue;
      }
      if (now_ms < sample.time_ms || now_ms - sample.time_ms > kBallTrackHistoryMaxAgeMs) {
        continue;
      }
      intercept_state_.history[write_index++] = sample;
    }
    for (int i = write_index; i < kBallTrackHistorySize; ++i) {
      intercept_state_.history[i] = BallTrackSample{};
    }
    intercept_state_.history_count = write_index;
  }

  void update_ball_track(const FootballVisionState& vision, int now_ms) {
    prune_ball_track_history(now_ms);
    double line_normal_mm = 0.0;
    double line_position_mm = 0.0;
    if (!resolve_ball_position_in_start_frame(vision, &line_normal_mm, &line_position_mm)) {
      return;
    }

    push_ball_track_sample(line_normal_mm, line_position_mm, now_ms);
    intercept_state_.last_seen_time_ms = now_ms;
  }

  void update_intercept_pose_estimate() {
    const double current_forward_mm = odometry().x_m * 1000.0;
    const double current_right_mm = odometry().y_m * 1000.0;
    if (!intercept_state_.pose_initialized) {
      intercept_state_.last_odom_forward_mm = current_forward_mm;
      intercept_state_.last_odom_strafe_mm = current_right_mm;
      intercept_state_.pose_initialized = true;
      return;
    }

    const double delta_forward_mm = current_forward_mm - intercept_state_.last_odom_forward_mm;
    const double delta_right_mm = current_right_mm - intercept_state_.last_odom_strafe_mm;
    intercept_state_.last_odom_forward_mm = current_forward_mm;
    intercept_state_.last_odom_strafe_mm = current_right_mm;

    if (std::fabs(delta_forward_mm) < 1e-3 && std::fabs(delta_right_mm) < 1e-3) {
      return;
    }

    const double current_heading_error_deg = std::fabs(
        normalize_angle_deg(local_inertial_heading_deg() -
                            intercept_pose_heading_deg(intercept_state_.target_pose)));

    if (current_heading_error_deg <= kInterceptPoseToleranceDeg + 1.0) {
      double delta_s_mm = 0.0;
      switch (intercept_state_.target_pose) {
        case InterceptPose::kLeft45:
        case InterceptPose::kRight45:
          delta_s_mm = delta_right_mm;
          break;
        case InterceptPose::kForward:
        default:
          delta_s_mm = 0.0;
          break;
      }
      intercept_state_.world_s_robot_mm += delta_s_mm;
    }
  }

  InterceptPrediction make_intercept_prediction(int now_ms) {
    InterceptPrediction prediction;
    prediction.valid = intercept_state_.history_count > 0;
    prediction.has_ball = prediction.valid;
    if (!prediction.valid) {
      return prediction;
    }

    const BallTrackSample& latest =
        intercept_state_.history[intercept_state_.history_count - 1];
    prediction.current_forward_mm = latest.x_mm;
    prediction.current_right_mm = latest.z_mm;
    prediction.intercept_right_mm =
        latest.z_mm;
    prediction.in_bounds =
        std::fabs(prediction.intercept_right_mm) <= kInterceptLineHalfWidthMm;
    prediction.in_bounds =
        std::fabs(prediction.intercept_right_mm) <= kInterceptLineHalfWidthMm;
    return prediction;
  }

  void run_intercept_scan_step(int now_ms) {
    prune_ball_track_history(now_ms);

    const double current_heading_deg = local_inertial_heading_deg();
    const double elapsed_ms = static_cast<double>(now_ms - intercept_state_.scan_phase_started_ms);
    if (elapsed_ms >= kInterceptScanHoldMs) {
      intercept_state_.scan_phase = (intercept_state_.scan_phase + 1) % 3;
      intercept_state_.scan_phase_started_ms = now_ms;
    }

    const InterceptPose scan_pose =
        intercept_state_.scan_phase == 0
            ? InterceptPose::kForward
            : (intercept_state_.scan_phase == 1 ? InterceptPose::kLeft45 : InterceptPose::kRight45);
    request_intercept_pose(scan_pose, now_ms);

    const double target_heading_deg = intercept_pose_heading_deg(scan_pose);
    const double heading_error_deg = normalize_angle_deg(target_heading_deg - current_heading_deg);
    double turn_pct = clamp_abs(
        heading_error_deg * kInterceptPoseTurnGainPctPerDeg * kAutoTurnDirectionSign,
        kInterceptPoseTurnMaxPct);
    if (std::fabs(heading_error_deg) > kInterceptPoseToleranceDeg) {
      turn_pct = signed_min_speed(turn_pct, kInterceptPoseTurnMinPct);
    }

    apply_drive_request(0.0, 0.0, turn_pct, vex::hold);
  }

  static void start_background_tasks() {
    current_football_robot_plus().run_background_tasks();
  }

  static void start_driver_control_entry() {
    current_football_robot_plus().run_driver_control_loop();
  }

  static void start_autonomous_entry() {
    current_football_robot_plus().run_autonomous_routine();
  }

  void run_background_tasks() {
    show_mode_status();
    while (true) {
      basic::input::controller_update(hardware_.brain, hardware_.controller, state_.controller);
      handle_vision_target_color_select();
      refresh_external_vision();
      handle_pose_readout();
      //show_current_screen();
      handle_auto_mode_toggle();
      handle_intercept_mode_toggle();

      if (auto_mode_ == AutoMode::kFaceTarget) {
        run_face_target_step();
      } else if (auto_mode_ == AutoMode::kIntercept) {
        run_intercept_step();
      } else if (should_accept_manual_control()) {
        run_manual_control_step();
      } else {
        stop_drive(vex::coast);
      }

      vex::this_thread::sleep_for(10);
    }
  }

  void run_driver_control_loop() {
    while (should_run_driver_control()) {
      vex::this_thread::sleep_for(kRefreshTime);
    }
  }

  void run_autonomous_routine() {
    while (should_run_autonomous_callback()) {
      vex::this_thread::sleep_for(kRefreshTime);
    }
  }

  bool should_run_driver_control() const {
    return competition_ != nullptr && competition_->isEnabled() && competition_->isDriverControl();
  }

  bool should_run_autonomous_callback() const {
    return competition_ != nullptr && competition_->isEnabled() && competition_->isAutonomous();
  }

  bool should_accept_manual_control() const {
    return competition_ == nullptr || !competition_->isEnabled() || competition_->isDriverControl();
  }

  void handle_vision_target_color_select() {
    return;
  }

  void handle_pose_readout() {
    if (!state_.controller.press_x) {
      return;
    }

    pose_readout_until_ms_ = hardware_.brain.timer(vex::timeUnits::msec) + kPoseReadoutHoldMs;
    hardware_.controller.Screen.clearScreen();
    hardware_.controller.Screen.setCursor(1, 1);
    if (!state_.vision.estimate_available || !state_.vision.last_estimate.valid) {
      hardware_.controller.Screen.print("OBJ: INVALID");
      hardware_.controller.Screen.setCursor(2, 1);
      hardware_.controller.Screen.print("NO VALID TARGET");
      return;
    }

    const basic::vision::Vec3 robot_position = camera_point_to_robot_frame(
        state_.vision.config.camera_extrinsics,
        state_.vision.last_estimate.position_camera_mm);
    hardware_.controller.Screen.print(
        "X:%5.0f Y:%5.0f",
        robot_position.x,
        robot_position.y);
    hardware_.controller.Screen.setCursor(2, 1);
    hardware_.controller.Screen.print("Z:%5.0f", robot_position.z);
  }

  void show_current_screen() {
    const int now_ms = hardware_.brain.timer(vex::timeUnits::msec);
    if (now_ms <= pose_readout_until_ms_) {
      return;
    }
    show_vision_status();
  }

  void handle_auto_mode_toggle() {
    if (!state_.controller.press_y) {
      return;
    }

    auto_mode_ = auto_mode_ == AutoMode::kFaceTarget ? AutoMode::kManual : AutoMode::kFaceTarget;
    stop_drive(auto_mode_ == AutoMode::kManual ? vex::coast : vex::hold);
    if (auto_mode_ == AutoMode::kFaceTarget) {
      reset_intercept_state();
    }
    show_mode_status();
  }

  void handle_intercept_mode_toggle() {
    if (!state_.controller.press_b) {
      return;
    }

    auto_mode_ = auto_mode_ == AutoMode::kIntercept ? AutoMode::kManual : AutoMode::kIntercept;
    stop_drive(auto_mode_ == AutoMode::kManual ? vex::coast : vex::hold);
    if (auto_mode_ == AutoMode::kIntercept) {
      reset_intercept_state();
    }
    show_mode_status();
  }

  void run_manual_control_step() {
    static vex::motor& fl_motor = basic::chassis::x_chassis_fl_motor(hardware_.football_chassis);
    static vex::motor& fr_motor = basic::chassis::x_chassis_fr_motor(hardware_.football_chassis);
    static vex::motor& bl_motor = basic::chassis::x_chassis_bl_motor(hardware_.football_chassis);
    static vex::motor& br_motor = basic::chassis::x_chassis_br_motor(hardware_.football_chassis);
    // L1/L2/R1/R2 按键边沿触发切换四角电机满转（调试用）
    if (state_.controller.press_l1) fl_test_spin_ = !fl_test_spin_;
    if (state_.controller.press_l2) fr_test_spin_ = !fr_test_spin_;
    if (state_.controller.press_r1) bl_test_spin_ = !bl_test_spin_;
    if (state_.controller.press_r2) br_test_spin_ = !br_test_spin_;

    if (fl_test_spin_) {
      printf("%.2f\n", fl_motor.velocity(vex::velocityUnits::pct));
      basic::control::velocitycontrol(fl_motor, 100.0);
    }
    if (fr_test_spin_) {
      printf("%.2f\n", fr_motor.velocity(vex::velocityUnits::pct));
      basic::control::velocitycontrol(fr_motor, 100.0);
    }
    if (bl_test_spin_) {
      printf("%.2f\n", bl_motor.velocity(vex::velocityUnits::pct));
      basic::control::velocitycontrol(bl_motor, 100.0);
    }
    if (br_test_spin_) {
      printf("%.2f\n", br_motor.velocity(vex::velocityUnits::pct));
      basic::control::velocitycontrol(br_motor, 100.0);
    }

    if (fl_test_spin_ || fr_test_spin_ || bl_test_spin_ || br_test_spin_) {
      return;
    }

    const basic::chassis::XChassisCommand command =
        basic::chassis::x_chassis_command_from_controller(
            state_.controller,
            basic::chassis::x_chassis_state(hardware_.football_chassis).stop_brake_type);
    basic::chassis::x_chassis_update(hardware_.football_chassis, command);
    //limit_drive_output();
  }

  void run_face_target_step() {
    const int now_ms = state_.controller.time_ms > 0
                           ? state_.controller.time_ms
                           : hardware_.brain.timer(vex::timeUnits::msec);
    const FootballVisionState vision = state_.vision;
    if (!has_recent_target(vision, now_ms)) {
      stop_drive(vex::hold);
      return;
    }

    const double image_width_px = resolve_image_width_px(vision);
    const double lateral_error_norm = resolve_lateral_error_norm(vision, image_width_px);
    const double heading_error_rad = resolve_heading_error_rad(vision);
    const bool has_heading_error = is_finite(heading_error_rad);

    if (has_heading_error &&
        std::fabs(heading_error_rad) <= kAutoPickupHeadingToleranceRad) {
      stop_drive(vex::hold);
      return;
    }

    double turn_pct = 0.0;
    if (has_heading_error) {
      turn_pct = clamp_abs(
          heading_error_rad * kAutoTurnGainPctPerRad * kAutoTurnDirectionSign,
          kAutoMaxTurnPct);
      if (std::fabs(heading_error_rad) > kAutoHeadingToleranceRad &&
          std::fabs(turn_pct) < kAutoMinTurnPct) {
        turn_pct = turn_pct >= 0.0 ? kAutoMinTurnPct : -kAutoMinTurnPct;
      }
    } else {
      turn_pct = clamp_abs(
          lateral_error_norm * (kAutoTurnGainPctPerRad * 0.8) * kAutoTurnDirectionSign,
          kAutoMaxTurnPct);
    }

    apply_drive_request(0.0, 0.0, turn_pct, vex::hold);
  }

  void run_intercept_step() {
    const int now_ms = hardware_.brain.timer(vex::timeUnits::msec);
    const FootballVisionState vision = state_.vision;
    update_intercept_pose_estimate();
    update_ball_track(vision, now_ms);

    const bool has_live_ball =
        vision.last_detection.has_detection && vision.class_filter_passed &&
        now_ms >= vision.last_update_time_ms &&
        now_ms - vision.last_update_time_ms <= kVisionStaleTimeoutMs;
    if (!has_live_ball) {
      run_intercept_scan_step(now_ms);
      return;
    }

    const InterceptPrediction prediction = make_intercept_prediction(now_ms);
    if (!prediction.valid) {
      stop_drive(vex::hold);
      return;
    }

    const double line_position_error_mm =
        prediction.current_right_mm - intercept_state_.world_s_robot_mm;
    InterceptPose desired_pose =
        line_position_error_mm >= kInterceptPoseSwitchHysteresisMm
            ? InterceptPose::kRight45
            : (line_position_error_mm <= -kInterceptPoseSwitchHysteresisMm
                   ? InterceptPose::kLeft45
                   : intercept_state_.target_pose);
    if (desired_pose == InterceptPose::kForward) {
      desired_pose = line_position_error_mm >= 0.0 ? InterceptPose::kRight45 : InterceptPose::kLeft45;
    }
    request_intercept_pose(desired_pose, now_ms);

    const double line_move_pct = signed_min_speed(
        clamp_abs(line_position_error_mm * kInterceptLineGainPctPerMm, kInterceptLineMaxPct),
        kInterceptLineMinPct);

    const double target_heading_deg = intercept_pose_heading_deg(intercept_state_.target_pose);
    const double heading_error_deg =
        normalize_angle_deg(target_heading_deg - local_inertial_heading_deg());
    double turn_pct = clamp_abs(
        heading_error_deg * kInterceptPoseTurnGainPctPerDeg * kAutoTurnDirectionSign,
        kInterceptPoseTurnMaxPct);
    if (std::fabs(heading_error_deg) > kInterceptPoseToleranceDeg) {
      turn_pct = signed_min_speed(turn_pct, kInterceptPoseTurnMinPct);
    }

    const bool pose_locked = std::fabs(heading_error_deg) <= kInterceptPoseToleranceDeg;
    if (!pose_locked) {
      apply_drive_request(0.0, 0.0, turn_pct, vex::hold);
      return;
    }

    if (std::fabs(line_position_error_mm) <= kInterceptLineAlignedToleranceMm) {
      apply_drive_request(0.0, 0.0, turn_pct, vex::hold);
      return;
    }

    apply_drive_request(0.0, line_move_pct, turn_pct, vex::hold);
  }

  bool has_recent_target(const FootballVisionState& vision, int now_ms) const {
    if (!vision.last_detection.has_detection || !vision.class_filter_passed) {
      return false;
    }

    if (now_ms < vision.last_update_time_ms ||
        now_ms - vision.last_update_time_ms > kVisionStaleTimeoutMs) {
      return false;
    }

    return vision.last_detection.bbox_px.valid() ||
           (vision.estimate_available && vision.last_estimate.valid);
  }

  double resolve_image_width_px(const FootballVisionState& vision) const {
    if (is_positive_finite(vision.last_detection.image_width_px)) {
      return vision.last_detection.image_width_px;
    }
    if (is_positive_finite(vision.config.image_width_px)) {
      return vision.config.image_width_px;
    }
    return 0.0;
  }

  double resolve_lateral_error_norm(
      const FootballVisionState& vision,
      double image_width_px) const {
    if (vision.estimate_available && vision.last_estimate.valid) {
      if (camera_extrinsics_valid(vision.config.camera_extrinsics)) {
        const basic::vision::Vec3 robot_ray = camera_vector_to_robot_frame(
            vision.config.camera_extrinsics,
            vision.last_estimate.ray_camera);
        if (basic::vision::is_finite(robot_ray.x)) {
          return clamp_abs(robot_ray.x, 1.0);
        }
      }

      if (basic::vision::is_finite(vision.last_estimate.ray_camera.x)) {
        return clamp_abs(vision.last_estimate.ray_camera.x, 1.0);
      }
    }

    if (vision.last_detection.bbox_px.valid() && image_width_px > 0.0) {
      const double image_center_px = image_width_px * 0.5;
      const double bbox_center_px =
          vision.last_detection.bbox_px.x + vision.last_detection.bbox_px.width * 0.5;
      return clamp_abs((bbox_center_px - image_center_px) / image_center_px, 1.0);
    }

    return 0.0;
  }

  double resolve_heading_error_rad(const FootballVisionState& vision) const {
    if (vision.estimate_available && vision.last_estimate.valid) {
      const basic::vision::Vec3 robot_position = camera_point_to_robot_frame(
          vision.config.camera_extrinsics,
          vision.last_estimate.position_camera_mm);
      if (is_finite(robot_position.x) && is_finite(robot_position.z) &&
          (std::fabs(robot_position.x) > 1e-6 || std::fabs(robot_position.z) > 1e-6)) {
        return std::atan2(robot_position.x, robot_position.z);
      }

      const basic::vision::Vec3 robot_ray = camera_vector_to_robot_frame(
          vision.config.camera_extrinsics,
          vision.last_estimate.ray_camera);
      if (is_finite(robot_ray.x) && is_finite(robot_ray.z) &&
          (std::fabs(robot_ray.x) > 1e-6 || std::fabs(robot_ray.z) > 1e-6)) {
        return std::atan2(robot_ray.x, robot_ray.z);
      }
    }

    if (vision.last_detection.bbox_px.valid()) {
      const basic::vision::CameraModel camera =
          resolve_camera_model(vision.config, vision.last_detection);
      const double bbox_center_x_px =
          vision.last_detection.bbox_px.x + vision.last_detection.bbox_px.width * 0.5;
      if (camera.valid() && is_finite(bbox_center_x_px)) {
        const basic::vision::Vec3 camera_ray{
            (bbox_center_x_px - camera.cx) / camera.fx,
            0.0,
            1.0,
        };
        const basic::vision::Vec3 robot_ray = camera_vector_to_robot_frame(
            vision.config.camera_extrinsics,
            camera_ray);
        if (is_finite(robot_ray.x) && is_finite(robot_ray.z) &&
            (std::fabs(robot_ray.x) > 1e-6 || std::fabs(robot_ray.z) > 1e-6)) {
          return std::atan2(robot_ray.x, robot_ray.z);
        }
      }
    }

    return basic::vision::nan_value();
  }

  double resolve_forward_distance_mm(const FootballVisionState& vision) const {
    if (!vision.estimate_available || !vision.last_estimate.valid) {
      return basic::vision::nan_value();
    }

    if (camera_extrinsics_valid(vision.config.camera_extrinsics)) {
      const basic::vision::Vec3 robot_position = camera_point_to_robot_frame(
          vision.config.camera_extrinsics,
          vision.last_estimate.position_camera_mm);
      if (is_positive_finite(robot_position.z)) {
        return robot_position.z;
      }
    }

    if (is_positive_finite(vision.last_estimate.position_camera_mm.z)) {
      return vision.last_estimate.position_camera_mm.z;
    }
    if (is_positive_finite(vision.last_estimate.depth_mm)) {
      return vision.last_estimate.depth_mm;
    }
    if (is_positive_finite(vision.last_estimate.range_mm)) {
      return vision.last_estimate.range_mm;
    }

    return basic::vision::nan_value();
  }

  /// 限制 X 底盘四角输出不超过预设上限
  void limit_drive_output() {
    const basic::chassis::XChassisState& state =
        basic::chassis::x_chassis_state(hardware_.football_chassis);
    const double drive_max_abs = std::max(
        {std::fabs(state.fl_pct), std::fabs(state.fr_pct),
         std::fabs(state.bl_pct), std::fabs(state.br_pct)});
    const double drive_scale =
        (drive_max_abs > kDriveOutputLimitPct && drive_max_abs > 0.0)
            ? (kDriveOutputLimitPct / drive_max_abs)
            : 1.0;

    if (drive_scale >= 1.0) {
      return;
    }

    basic::chassis::x_drive_set_output(
        hardware_.football_chassis,
        state.fl_pct * drive_scale,
        state.fr_pct * drive_scale,
        state.bl_pct * drive_scale,
        state.br_pct * drive_scale,
        state.stop_brake_type);
  }

  /// 使用 X-drive mecanum 运动学分解前后/平移/旋转指令
  void apply_drive_request(
      double forward_pct,
      double strafe_pct,
      double turn_pct,
      vex::brakeType brake_type) {
    // X-drive mecanum 运动学分解：
    // fl = forward + strafe + turn
    // fr = forward - strafe - turn
    // bl = forward - strafe + turn
    // br = forward + strafe - turn
    double fl_pct = forward_pct + strafe_pct + turn_pct;
    double fr_pct = forward_pct - strafe_pct - turn_pct;
    double bl_pct = forward_pct - strafe_pct + turn_pct;
    double br_pct = forward_pct + strafe_pct - turn_pct;

    const double max_pct = std::max(
        {std::fabs(fl_pct), std::fabs(fr_pct),
         std::fabs(bl_pct), std::fabs(br_pct)});
    if (max_pct > 100.0) {
      const double scale = 100.0 / max_pct;
      fl_pct *= scale;
      fr_pct *= scale;
      bl_pct *= scale;
      br_pct *= scale;
    }

    basic::chassis::x_drive_set_output(
        hardware_.football_chassis,
        fl_pct,
        fr_pct,
        bl_pct,
        br_pct,
        brake_type);
    limit_drive_output();
  }

  void stop_drive(vex::brakeType drive_brake_type) {
    basic::chassis::x_chassis_stop(hardware_.football_chassis, drive_brake_type);
  }

  void refresh_external_vision() {
    ExternalVisionPacket packet;
    const bool any_update = hardware_.external_vision.poll(&packet);
    state_.vision.external_link = hardware_.external_vision.link_state();
    state_.vision.last_blob_detection.sensor_installed = state_.vision.external_link.online;

    if (packet.has_observation_update) {
      set_vision_target_color(packet.target_color);
      submit_yolo_detection(packet.detection);
    } else if (!has_fresh_external_detection(
                   state_.vision,
                   hardware_.brain.timer(vex::timeUnits::msec))) {
      clear_yolo_detection();
    }

    if (!any_update) {
      state_.vision.external_link = hardware_.external_vision.link_state();
      state_.vision.last_blob_detection.sensor_installed = state_.vision.external_link.online;
    }
  }

  void show_vision_status() {
    const FootballVisionState& vision = state_.vision;
    const bool online = vision.external_link.online;

    hardware_.controller.Screen.setCursor(1, 1);
    if (auto_mode_ == AutoMode::kIntercept) {
      hardware_.controller.Screen.print("MODE: INTERCEPT   ");
    } else if (auto_mode_ == AutoMode::kFaceTarget) {
      hardware_.controller.Screen.print("MODE: FACE TARGET ");
    } else {
      hardware_.controller.Screen.print("MODE: MANUAL      ");
    }

    if (!online) {
      hardware_.controller.Screen.setCursor(2, 1);
      hardware_.controller.Screen.print("VISION: SERIAL OFF ");
      hardware_.controller.Screen.setCursor(3, 1);
      hardware_.controller.Screen.print(
          "ERR:%2d TS:%4d ",
          vision.external_link.parse_error_count,
          vision.external_link.last_source_timestamp_ms);
      return;
    }

    hardware_.controller.Screen.setCursor(2, 1);
    hardware_.controller.Screen.print(
        "T:%4d C:%c I:%1d ",
        vision.last_detection.source_timestamp_ms,
        vision.external_link.reported_color_code,
        vision.last_detection.class_id);

    hardware_.controller.Screen.setCursor(3, 1);
    if (auto_mode_ == AutoMode::kIntercept) {
      hardware_.controller.Screen.print(
          "S:%5.0f B:%5.0f %c ",
          intercept_state_.world_s_robot_mm,
          intercept_state_.history_count > 0
              ? intercept_state_.history[intercept_state_.history_count - 1].z_mm
              : 0.0,
          intercept_state_.target_pose == InterceptPose::kLeft45
              ? 'L'
              : (intercept_state_.target_pose == InterceptPose::kRight45 ? 'R' : 'F'));
      return;
    }

    hardware_.controller.Screen.print(
        "X:%3d Y:%3d    ",
        static_cast<int>(vision.last_detection.bbox_px.x),
        static_cast<int>(vision.last_detection.bbox_px.y));
  }

  void show_mode_status() {
    show_vision_status();
  }

  RobotHardware hardware_;
  RobotState state_;
  basic::vision::MonocularLocator locator_;
  vex::competition* competition_{nullptr};
  AutoMode auto_mode_{AutoMode::kManual};
  InterceptState intercept_state_{};
  bool fl_test_spin_{false};
  bool fr_test_spin_{false};
  bool bl_test_spin_{false};
  bool br_test_spin_{false};
  int pose_readout_until_ms_{0};

  friend FootballRobotPlus& current_football_robot_plus();
};

FootballRobotPlus& current_football_robot_plus() {
  static FootballRobotPlus robot;
  return robot;
}

}  // namespace

basic::app::Robot& get_robot() {
  return current_football_robot_plus();
}

void configure_vision(const FootballVisionConfig& config) {
  current_football_robot_plus().configure_vision(config);
}

void set_vision_target_color(basic::identify::VisionTargetColor color) {
  current_football_robot_plus().set_vision_target_color(color);
}

basic::identify::VisionTargetColor get_vision_target_color() {
  return current_football_robot_plus().vision_target_color();
}

basic::identify::LargestBlobDetection get_vision_sensor_detection() {
  return current_football_robot_plus().vision_sensor_detection();
}

basic::vision::EstimateResult submit_yolo_detection(const YoloDetection& detection) {
  return current_football_robot_plus().submit_yolo_detection(detection);
}

void clear_yolo_detection() {
  current_football_robot_plus().clear_yolo_detection();
}

FootballVisionState get_vision_state() {
  return current_football_robot_plus().vision_state();
}

}  // namespace basic::hardware::football_robot_plus
