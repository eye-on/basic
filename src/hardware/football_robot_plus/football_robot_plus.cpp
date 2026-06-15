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
inline constexpr int kInterceptTargetAcquireFrames = 2;
inline constexpr int kInterceptTargetLossHoldMs = 180;
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
inline constexpr int kCameraGimbalInputDeadzonePct = 5;
inline constexpr double kGimbalScanLimitDeg = 45.0;
inline constexpr double kGimbalScanSpeedPct = 18.0;
inline constexpr double kGimbalTrackToleranceDeg = 2.0;
inline constexpr double kGimbalTrackGainPctPerDeg = 2.2;
inline constexpr double kGimbalTrackMinPct = 8.0;
inline constexpr double kGimbalTrackMaxPct = 35.0;
inline constexpr double kGimbalTrackDirectionSign = -1.0;
inline constexpr double kGimbalTrackRampPctPerStep = 4.0;
inline constexpr double kInterceptGimbalCenterToleranceDeg = 5.0;
inline constexpr double kInterceptStrafeGainPctPerDeg = 20.0;
inline constexpr double kInterceptStrafeMinPct = 14.0;
inline constexpr double kInterceptStrafeMaxPct = 100.0;
inline constexpr double kInterceptStrafeDirectionSign = 1.0;
inline constexpr double kInterceptStrafeRampPctPerStep = 6.0;
inline constexpr double kInterceptHeadingHoldToleranceDeg = 2.0;
inline constexpr double kInterceptHeadingHoldGainPctPerDeg = 1.0;
inline constexpr double kInterceptHeadingHoldMinPct = 8.0;
inline constexpr double kInterceptHeadingHoldMaxPct = 24.0;
inline constexpr double kFallbackImageHalfFovDeg = 30.0;
inline constexpr double kInterceptLineHalfWidthMm = 900.0;
inline constexpr int kExternalVisionStaleTimeoutMs = 500;
inline constexpr int kPoseReadoutHoldMs = 1500;

enum class AutoMode {
  kManual,
  kFaceTarget,
  kIntercept,
};

struct InterceptState {
  int scan_phase_started_ms{0};
  double line_heading_deg{0.0};
  double line_origin_right_mm{0.0};
  double gimbal_zero_deg{0.0};
  int gimbal_scan_direction{1};
  int last_vision_update_time_ms{-1};
  int consecutive_target_frames{0};
  int last_valid_target_time_ms{0};
  bool target_locked{false};
  double gimbal_command_pct{0.0};
  double strafe_command_pct{0.0};
};

double clamp_value(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

double clamp_abs(double value, double max_abs) {
  return clamp_value(value, -max_abs, max_abs);
}

double ramp_toward(double current, double target, double max_step) {
  if (max_step <= 0.0) {
    return target;
  }
  if (target > current) {
    return std::min(current + max_step, target);
  }
  return std::max(current - max_step, target);
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
  config.camera_extrinsics.yaw_deg = 0.0;
  return config;
}

class FootballRobotPlus;
FootballRobotPlus& current_football_robot_plus();

class FootballRobotPlus final : public basic::app::Robot {
 public:
  void initialize() override {
    configure_vision(default_vision_config_for_sensor());
    sync_camera_gimbal_state();
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
    sync_camera_gimbal_state();
    intercept_state_ = InterceptState{};
    intercept_state_.line_heading_deg = local_inertial_heading_deg();
    intercept_state_.scan_phase_started_ms = hardware_.brain.timer(vex::timeUnits::msec);
    intercept_state_.line_origin_right_mm = odometry().y_m * 1000.0;
    intercept_state_.gimbal_zero_deg = state_.camera_gimbal.motor_position_deg;
    intercept_state_.gimbal_scan_direction = 1;
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
    while (true) {
      basic::input::controller_update(hardware_.brain, hardware_.controller, state_.controller);
      handle_vision_target_color_select();
      refresh_external_vision();
      handle_auto_mode_toggle();
      handle_intercept_mode_toggle();
      if (auto_mode_ == AutoMode::kManual && should_accept_manual_control()) {
        run_camera_gimbal_manual_step();
      } else if (auto_mode_ != AutoMode::kIntercept) {
        stop_camera_gimbal(vex::hold);
      }

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
    return;
  }

  void show_current_screen() {
    return;
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
  }

  void handle_intercept_mode_toggle() {
    if (!state_.controller.press_b) {
      return;
    }

    auto_mode_ = auto_mode_ == AutoMode::kIntercept ? AutoMode::kManual : AutoMode::kIntercept;
    intercept_debug_print_enabled_ = auto_mode_ == AutoMode::kIntercept;
    stop_drive(auto_mode_ == AutoMode::kManual ? vex::coast : vex::hold);
    if (auto_mode_ == AutoMode::kIntercept) {
      reset_intercept_state();
    }
  }

  void run_camera_gimbal_manual_step() {
    const int axis_input_pct = state_.controller.axis3;
    if (std::abs(axis_input_pct) < kCameraGimbalInputDeadzonePct) {
      basic::mechanism::camera_gimbal_stop(hardware_.camera_gimbal, vex::hold);
      sync_camera_gimbal_state();
      return;
    }

    basic::mechanism::camera_gimbal_set_output(
        hardware_.camera_gimbal,
        static_cast<double>(axis_input_pct));
    sync_camera_gimbal_state();
  }

  void run_manual_control_step() {
    static vex::motor& fl_motor = basic::chassis::x_chassis_fl_motor(hardware_.football_chassis);
    static vex::motor& fr_motor = basic::chassis::x_chassis_fr_motor(hardware_.football_chassis);
    static vex::motor& bl_motor = basic::chassis::x_chassis_bl_motor(hardware_.football_chassis);
    static vex::motor& br_motor = basic::chassis::x_chassis_br_motor(hardware_.football_chassis);
    // L1/L2/R1/R2 ????????????��??????????????????
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
    sync_camera_gimbal_state();
    update_intercept_target_lock(vision, now_ms);
    print_intercept_debug(now_ms, vision);

    const bool has_live_ball = has_live_intercept_target();
    const bool has_target_measurement = has_current_intercept_target_measurement(vision, now_ms);
    if (!has_live_ball) {
      run_intercept_gimbal_scan_step();
      intercept_state_.strafe_command_pct = 0.0;
      // 没有目标时只扫云台，底盘保持静止。
      stop_drive(vex::hold);
      return;
    }

    run_intercept_gimbal_track_step(vision, has_target_measurement);
    apply_drive_request(
        0.0,
        make_intercept_strafe_pct_from_gimbal(),
        make_intercept_heading_hold_turn_pct(),
        vex::hold);
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

  bool has_current_intercept_target_measurement(
      const FootballVisionState& vision,
      int now_ms) const {
    return vision.last_detection.has_detection && vision.class_filter_passed &&
           vision.last_detection.bbox_px.valid() &&
           now_ms >= vision.last_update_time_ms &&
           now_ms - vision.last_update_time_ms <= kVisionStaleTimeoutMs;
  }

  void update_intercept_target_lock(const FootballVisionState& vision, int now_ms) {
    if (vision.last_update_time_ms != intercept_state_.last_vision_update_time_ms) {
      intercept_state_.last_vision_update_time_ms = vision.last_update_time_ms;
      if (has_current_intercept_target_measurement(vision, now_ms)) {
        intercept_state_.last_valid_target_time_ms = vision.last_update_time_ms;
        intercept_state_.consecutive_target_frames =
            std::min(intercept_state_.consecutive_target_frames + 1,
                     kInterceptTargetAcquireFrames);
        if (intercept_state_.consecutive_target_frames >=
            kInterceptTargetAcquireFrames) {
          intercept_state_.target_locked = true;
        }
      } else {
        intercept_state_.consecutive_target_frames = 0;
      }
    }

    if (!intercept_state_.target_locked) {
      return;
    }

    if (now_ms < intercept_state_.last_valid_target_time_ms ||
        now_ms - intercept_state_.last_valid_target_time_ms >
            kInterceptTargetLossHoldMs) {
      intercept_state_.target_locked = false;
      intercept_state_.consecutive_target_frames = 0;
    }
  }

  bool has_live_intercept_target() const {
    return intercept_state_.target_locked;
  }

  double current_intercept_line_position_mm() {
    return odometry().y_m * 1000.0 - intercept_state_.line_origin_right_mm;
  }

  double current_gimbal_relative_deg() {
    return state_.camera_gimbal.motor_position_deg - intercept_state_.gimbal_zero_deg;
  }

  double resolve_target_bearing_deg(const FootballVisionState& vision) {
    if (!vision.last_detection.bbox_px.valid()) {
      return basic::vision::nan_value();
    }

    const double bbox_center_x_px =
        vision.last_detection.bbox_px.x + vision.last_detection.bbox_px.width * 0.5;
    const basic::vision::CameraModel camera =
        resolve_camera_model(vision.config, vision.last_detection);
    if (camera.valid() && is_finite(bbox_center_x_px) && is_finite(camera.cx) &&
        is_finite(camera.fx) && std::fabs(camera.fx) > 1e-6) {
      return radians_to_degrees(std::atan((bbox_center_x_px - camera.cx) / camera.fx));
    }

    const double image_width_px = resolve_image_width_px(vision);
    if (!is_positive_finite(image_width_px) || !is_finite(bbox_center_x_px)) {
      return basic::vision::nan_value();
    }

    const double image_center_px = image_width_px * 0.5;
    return ((bbox_center_x_px - image_center_px) / image_center_px) *
           kFallbackImageHalfFovDeg;
  }

  double make_intercept_heading_hold_turn_pct() {
    const double heading_error_deg =
        normalize_angle_deg(intercept_state_.line_heading_deg - local_inertial_heading_deg());
    if (std::fabs(heading_error_deg) <= kInterceptHeadingHoldToleranceDeg) {
      return 0.0;
    }

    double turn_pct = clamp_abs(
        heading_error_deg * kInterceptHeadingHoldGainPctPerDeg * kAutoTurnDirectionSign,
        kInterceptHeadingHoldMaxPct);
    if (std::fabs(turn_pct) < kInterceptHeadingHoldMinPct) {
      turn_pct = turn_pct >= 0.0 ? kInterceptHeadingHoldMinPct : -kInterceptHeadingHoldMinPct;
    }
    return turn_pct;
  }

  void run_intercept_gimbal_scan_step() {
    double relative_deg = current_gimbal_relative_deg();
    if (relative_deg >= kGimbalScanLimitDeg) {
      intercept_state_.gimbal_scan_direction = -1;
    } else if (relative_deg <= -kGimbalScanLimitDeg) {
      intercept_state_.gimbal_scan_direction = 1;
    }

    set_intercept_gimbal_output_pct(
        static_cast<double>(intercept_state_.gimbal_scan_direction) * kGimbalScanSpeedPct);
  }

  void run_intercept_gimbal_track_step(
      const FootballVisionState& vision,
      bool has_target_measurement) {
    if (!has_target_measurement) {
      set_intercept_gimbal_output_pct(0.0);
      return;
    }

    const double bearing_deg = resolve_target_bearing_deg(vision);
    if (!is_finite(bearing_deg) || std::fabs(bearing_deg) <= kGimbalTrackToleranceDeg) {
      set_intercept_gimbal_output_pct(0.0);
      return;
    }

    double motor_pct = clamp_abs(
        bearing_deg * kGimbalTrackGainPctPerDeg * kGimbalTrackDirectionSign,
        kGimbalTrackMaxPct);
    if (std::fabs(motor_pct) < kGimbalTrackMinPct) {
      motor_pct = motor_pct >= 0.0 ? kGimbalTrackMinPct : -kGimbalTrackMinPct;
    }

    set_intercept_gimbal_output_pct(motor_pct);
  }

  void set_intercept_gimbal_output_pct(double target_pct) {
    intercept_state_.gimbal_command_pct = ramp_toward(
        intercept_state_.gimbal_command_pct,
        clamp_abs(target_pct, 100.0),
        kGimbalTrackRampPctPerStep);
    basic::mechanism::camera_gimbal_set_output(
        hardware_.camera_gimbal,
        intercept_state_.gimbal_command_pct);
    sync_camera_gimbal_state();
  }

  double make_intercept_strafe_pct_from_gimbal() {
    const double gimbal_error_deg = current_gimbal_relative_deg();
    if (std::fabs(gimbal_error_deg) <= kInterceptGimbalCenterToleranceDeg) {
      intercept_state_.strafe_command_pct = ramp_toward(
          intercept_state_.strafe_command_pct,
          0.0,
          kInterceptStrafeRampPctPerStep);
      return intercept_state_.strafe_command_pct;
    }

    double target_strafe_pct = clamp_abs(
        gimbal_error_deg * kInterceptStrafeGainPctPerDeg * kInterceptStrafeDirectionSign,
        kInterceptStrafeMaxPct);
    if (std::fabs(target_strafe_pct) < kInterceptStrafeMinPct) {
      target_strafe_pct = target_strafe_pct >= 0.0 ? kInterceptStrafeMinPct
                                                   : -kInterceptStrafeMinPct;
    }

    const double line_position_mm = current_intercept_line_position_mm();
    if ((line_position_mm >= kInterceptLineHalfWidthMm && target_strafe_pct > 0.0) ||
        (line_position_mm <= -kInterceptLineHalfWidthMm && target_strafe_pct < 0.0)) {
      target_strafe_pct = 0.0;
    }

    intercept_state_.strafe_command_pct = ramp_toward(
        intercept_state_.strafe_command_pct,
        target_strafe_pct,
        kInterceptStrafeRampPctPerStep);
    return intercept_state_.strafe_command_pct;
  }

  /// ???? X ???????????????????????
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

  /// ??? X-drive mecanum ??????????/???/??????
  void apply_drive_request(
      double forward_pct,
      double strafe_pct,
      double turn_pct,
      vex::brakeType brake_type) {
    // X-drive mecanum ???????
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
    // limit_drive_output();
  }

  void stop_drive(vex::brakeType drive_brake_type) {
    basic::chassis::x_chassis_stop(hardware_.football_chassis, drive_brake_type);
  }

  void stop_camera_gimbal(vex::brakeType brake_type) {
    basic::mechanism::camera_gimbal_stop(hardware_.camera_gimbal, brake_type);
    sync_camera_gimbal_state();
  }

  void sync_camera_gimbal_state() {
    basic::mechanism::camera_gimbal_refresh_state(hardware_.camera_gimbal);
    state_.camera_gimbal = basic::mechanism::camera_gimbal_state(hardware_.camera_gimbal);
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
    return;
  }

  void show_mode_status() {
    return;
  }

  void print_intercept_debug(int now_ms, const FootballVisionState& vision) {
    if (!intercept_debug_print_enabled_ || auto_mode_ != AutoMode::kIntercept) {
      return;
    }

    const bool has_ball = has_live_intercept_target();
    const bool has_measurement = has_current_intercept_target_measurement(vision, now_ms);
    printf(
        "ts=%d S=%.1f B=%.1f ball=%d seen=%d gp=%.1f sp=%.1f\n",
        now_ms,
        current_intercept_line_position_mm(),
        current_gimbal_relative_deg(),
        has_ball ? 1 : 0,
        has_measurement ? 1 : 0,
        intercept_state_.gimbal_command_pct,
        intercept_state_.strafe_command_pct);
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
  bool intercept_debug_print_enabled_{false};
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