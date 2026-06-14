#include "hardware/robot_selector.h"

#include "chassis/x_chassis.h"
#include "chassis/x_drive.h"
#include "hardware/football_robot_plus/external_vision_serial.h"
#include "hardware/football_robot_plus/robot_hardware.h"
#include "hardware/football_robot_plus/robot_state.h"
#include "hardware/football_robot_plus/vision.h"
#include "input/controller.h"

#include <algorithm>
#include <cmath>

namespace basic::hardware::football_robot_plus {

namespace {

inline constexpr int kBackgroundLoopDelayMs = kRefreshTime;
inline constexpr double kEstimatedPositionYOffsetMm = 30.0;
inline constexpr int kVisionStaleTimeoutMs = 500;
inline constexpr double kAutoCenterToleranceNorm = 0.08;
inline constexpr double kAutoPickupCenterToleranceNorm = 0.05;
inline constexpr double kAutoTargetRangeMm = 220.0;
inline constexpr double kAutoPickupRangeMm = 180.0;
inline constexpr double kAutoForwardGainPctPerMm = 0.04;
inline constexpr double kAutoStrafeGainPct = 70.0;
inline constexpr double kAutoMaxForwardPct = 25.0;
inline constexpr double kAutoMaxStrafePct = 100.0;
inline constexpr int kExternalVisionStaleTimeoutMs = 500;
inline constexpr int kPoseReadoutHoldMs = 1500;

double clamp_value(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

double clamp_abs(double value, double max_abs) {
  return clamp_value(value, -max_abs, max_abs);
}

bool is_positive_finite(double value) {
  return basic::vision::is_finite(value) && value > 0.0;
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
  config.camera_extrinsics.x_mm = 0.0;
  config.camera_extrinsics.y_mm = 0.0;
  config.camera_extrinsics.z_mm = 0.0;
  config.camera_extrinsics.roll_deg = 0.0;
  config.camera_extrinsics.pitch_deg = 0.0;
  config.camera_extrinsics.yaw_deg = 0.0;
  return config;
}

class FootballRobotPlus;
FootballRobotPlus& current_football_robot_plus();

class FootballRobotPlus final : public basic::app::Robot {
 public:
  void initialize() override {
    configure_vision(default_vision_config_for_sensor());
    hardware_.calibrate_inertial_sensor();
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
      show_current_screen();
      handle_auto_mode_toggle();

      if (auto_mode_enabled_) {
        run_resident_autonomous_step();
      } else if (should_accept_manual_control()) {
        run_manual_control_step();
      } else {
        stop_drive(vex::coast);
      }

      vex::this_thread::sleep_for(kBackgroundLoopDelayMs);
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

    auto_mode_enabled_ = !auto_mode_enabled_;
    stop_drive(auto_mode_enabled_ ? vex::hold : vex::coast);
    show_mode_status();
  }

  void run_manual_control_step() {
    const basic::chassis::XChassisCommand command =
        basic::chassis::x_chassis_command_from_controller(
            state_.controller,
            basic::chassis::x_chassis_state(hardware_.football_chassis).stop_brake_type);
    basic::chassis::x_chassis_update(hardware_.football_chassis, command);
    limit_drive_output();
  }

  void run_resident_autonomous_step() {
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
    const double forward_distance_mm = resolve_forward_distance_mm(vision);
    const bool has_forward_distance = is_positive_finite(forward_distance_mm);

    // 到达捡球范围后停止
    if (has_forward_distance &&
        forward_distance_mm <= kAutoPickupRangeMm &&
        std::fabs(lateral_error_norm) <= kAutoPickupCenterToleranceNorm) {
      stop_drive(vex::hold);
      return;
    }

    double forward_pct = 0.0;
    if (has_forward_distance) {
      forward_pct = clamp_value(
          (forward_distance_mm - kAutoTargetRangeMm) * kAutoForwardGainPctPerMm,
          0.0,
          kAutoMaxForwardPct);
      // 未对准时限制前进速度，优先完成横向对准
      if (std::fabs(lateral_error_norm) > kAutoCenterToleranceNorm) {
        forward_pct = std::min(forward_pct, 10.0);
      }
    }

    const double strafe_pct =
        clamp_abs(lateral_error_norm * kAutoStrafeGainPct, kAutoMaxStrafePct);
    apply_drive_request(forward_pct, strafe_pct, 0.0, vex::hold);
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

    if (!online) {
      hardware_.controller.Screen.setCursor(1, 1);
      hardware_.controller.Screen.print("VISION: SERIAL OFF ");
      hardware_.controller.Screen.setCursor(2, 1);
      hardware_.controller.Screen.print(
          "ERR:%2d TS:%4d ",
          vision.external_link.parse_error_count,
          vision.external_link.last_source_timestamp_ms);
      hardware_.controller.Screen.setCursor(3, 1);
      hardware_.controller.Screen.print("WAIT OBS FRAME    ");
      return;
    }

    hardware_.controller.Screen.setCursor(1, 1);
    hardware_.controller.Screen.print(
        "T:%4d C:%c I:%1d ",
        vision.last_detection.source_timestamp_ms,
        vision.external_link.reported_color_code,
        vision.last_detection.class_id);

    hardware_.controller.Screen.setCursor(2, 1);
    hardware_.controller.Screen.print(
        "X:%3d Y:%3d    ",
        static_cast<int>(vision.last_detection.bbox_px.x),
        static_cast<int>(vision.last_detection.bbox_px.y));

    hardware_.controller.Screen.setCursor(3, 1);
    if (!vision.last_detection.has_detection || !vision.last_detection.bbox_px.valid()) {
      hardware_.controller.Screen.print(
          "W:%3d H:%3d D:0",
          static_cast<int>(vision.last_detection.bbox_px.width),
          static_cast<int>(vision.last_detection.bbox_px.height));
      return;
    }

    hardware_.controller.Screen.print(
        "W:%3d H:%3d S:%2.0f",
        static_cast<int>(vision.last_detection.bbox_px.width),
        static_cast<int>(vision.last_detection.bbox_px.height),
        vision.last_detection.score * 100.0);
  }

  void show_mode_status() {
    show_vision_status();
  }

  RobotHardware hardware_;
  RobotState state_;
  basic::vision::MonocularLocator locator_;
  vex::competition* competition_{nullptr};
  bool auto_mode_enabled_{false};
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
