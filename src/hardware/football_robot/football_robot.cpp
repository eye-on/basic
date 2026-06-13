#include "hardware/robot_selector.h"

#include "chassis/h_chassis.h"
#include "chassis/h_drive.h"
#include "hardware/football_robot/robot_hardware.h"
#include "hardware/football_robot/robot_state.h"
#include "hardware/football_robot/vision.h"
#include "input/controller.h"
#include "mechanism/pneumatic_motor_actuator.h"

#include <algorithm>
#include <cmath>

namespace basic::hardware::football_robot {

namespace {

inline constexpr int kBackgroundLoopDelayMs = kRefreshTime;
inline constexpr int kVisionStaleTimeoutMs = 500;
inline constexpr double kAutoCenterToleranceNorm = 0.08;
inline constexpr double kAutoPickupCenterToleranceNorm = 0.05;
inline constexpr double kAutoTargetRangeMm = 220.0;
inline constexpr double kAutoPickupRangeMm = 180.0;
inline constexpr double kAutoForwardGainPctPerMm = 0.04;
inline constexpr double kAutoStrafeGainPct = 70.0;
inline constexpr double kAutoCenteringForwardLimitPct = 10.0;
inline constexpr double kAutoMaxForwardPct = 25.0;
inline constexpr double kAutoMaxStrafePct = 25.0;

double clamp_value(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

double clamp_abs(double value, double max_abs) {
  return clamp_value(value, -max_abs, max_abs);
}

bool is_positive_finite(double value) {
  return basic::vision::is_finite(value) && value > 0.0;
}

class FootballRobot;
FootballRobot& current_football_robot();

class FootballRobot final : public basic::app::Robot {
 public:
  void initialize() override {
    configure_vision(FootballVisionConfig{});
    hardware_.calibrate_inertial_sensor();
    sync_actuator_state();
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
    state_.vision = FootballVisionState{};
    state_.vision.config = config;
    locator_.set_config(config.estimator);
    state_.vision.last_update_time_ms = hardware_.brain.timer(vex::timeUnits::msec);
  }

  basic::vision::EstimateResult submit_yolo_detection(const YoloDetection& detection) {
    state_.vision.last_detection = detection;
    state_.vision.class_filter_passed =
        !detection.has_detection || state_.vision.config.expected_class_id < 0 ||
        detection.class_id == state_.vision.config.expected_class_id;
    state_.vision.last_update_time_ms = hardware_.brain.timer(vex::timeUnits::msec);

    if (!detection.has_detection) {
      state_.vision.last_estimate = basic::vision::EstimateResult{};
      state_.vision.estimate_available = false;
      return state_.vision.last_estimate;
    }

    state_.vision.last_estimate =
        basic::hardware::football_robot::estimate_football_from_yolo(
            locator_, state_.vision.config, detection);
    state_.vision.estimate_available = true;
    return state_.vision.last_estimate;
  }

  void clear_yolo_detection() {
    state_.vision.last_detection = YoloDetection{};
    state_.vision.last_estimate = basic::vision::EstimateResult{};
    state_.vision.estimate_available = false;
    state_.vision.class_filter_passed = true;
    state_.vision.last_update_time_ms = hardware_.brain.timer(vex::timeUnits::msec);
  }

  FootballVisionState vision_state() const { return state_.vision; }

  basic::mechanism::PneumaticMotorActuatorState actuator_state() const { return state_.actuator; }

  basic::mechanism::PneumaticMotorActuatorState refresh_actuator_state() {
    sync_actuator_state();
    return state_.actuator;
  }

  void set_actuator_motor_angle_state(basic::mechanism::MotorPositionState target_state) {
    basic::mechanism::pneumatic_motor_actuator_set_motor_target_state(
        hardware_.actuator,
        target_state);
    sync_actuator_state();
  }

  void toggle_actuator_motor_angle_state() {
    basic::mechanism::pneumatic_motor_actuator_toggle_motor_target_state(hardware_.actuator);
    sync_actuator_state();
  }

  void update_actuator_motor_to_target() {
    basic::mechanism::pneumatic_motor_actuator_update_motor_target(hardware_.actuator);
    sync_actuator_state();
  }

 private:
  static void start_background_tasks() {
    current_football_robot().run_background_tasks();
  }

  static void start_driver_control_entry() {
    current_football_robot().run_driver_control_loop();
  }

  static void start_autonomous_entry() {
    current_football_robot().run_autonomous_routine();
  }

  void run_background_tasks() {
    show_mode_status();
    while (true) {
      basic::input::controller_update(hardware_.brain, hardware_.controller, state_.controller);
      handle_test_readout();
      handle_auto_mode_toggle();

      if (auto_mode_enabled_) {
        run_resident_autonomous_step();
      } else if (should_accept_manual_control()) {
        run_manual_control_step();
      } else {
        stop_drive_and_actuator_motion(vex::coast);
      }

      vex::this_thread::sleep_for(kBackgroundLoopDelayMs);
    }
  }

  // Competition callbacks remain registered, but the resident background task
  // now owns the actual control loop.
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

  void handle_test_readout() {
    if (!state_.controller.press_b) {
      return;
    }

    const double motor_position_deg = hardware_.actuator.motor().position(vex::deg);
    state_.actuator.motor_position_deg = motor_position_deg;
    hardware_.controller.Screen.setCursor(2, 1);
    hardware_.controller.Screen.print("MTR[B]: %7.2f deg   ", motor_position_deg);
    printf("football actuator motor position: %.2f deg\n", motor_position_deg);
  }

  void handle_auto_mode_toggle() {
    if (!state_.controller.press_y) {
      return;
    }

    auto_mode_enabled_ = !auto_mode_enabled_;
    auto_pickup_triggered_ = false;
    stop_drive_and_actuator_motion(auto_mode_enabled_ ? vex::hold : vex::coast);
    show_mode_status();
  }

  void run_manual_control_step() {
    const basic::chassis::HChassisCommand command =
        basic::chassis::h_chassis_command_from_controller(
            state_.controller,
            basic::chassis::h_chassis_state(hardware_.football_chassis).stop_brake_type);
    basic::chassis::h_chassis_update(hardware_.football_chassis, command);
    basic::mechanism::pneumatic_motor_actuator_update(
        hardware_.actuator,
        basic::mechanism::pneumatic_motor_actuator_command_from_controller(
            state_.controller));
    sync_actuator_state();
    limit_drive_output();
  }

  void run_resident_autonomous_step() {
    const int now_ms = state_.controller.time_ms > 0
                           ? state_.controller.time_ms
                           : hardware_.brain.timer(vex::timeUnits::msec);
    const FootballVisionState vision = state_.vision;
    if (!has_recent_target(vision, now_ms)) {
      auto_pickup_triggered_ = false;
      stop_drive_and_actuator_motion(vex::hold);
      return;
    }

    const double image_width_px = resolve_image_width_px(vision);
    const double lateral_error_norm = resolve_lateral_error_norm(vision, image_width_px);
    const double forward_distance_mm = resolve_forward_distance_mm(vision);
    const bool has_forward_distance = is_positive_finite(forward_distance_mm);

    if (auto_pickup_triggered_) {
      stop_drive_and_actuator_motion(vex::hold);
      return;
    }

    if (has_forward_distance &&
        forward_distance_mm <= kAutoPickupRangeMm &&
        std::fabs(lateral_error_norm) <= kAutoPickupCenterToleranceNorm) {
      set_actuator_pneumatic_open(true);
      auto_pickup_triggered_ = true;
      stop_drive_and_actuator_motion(vex::hold);
      return;
    }

    double forward_pct = 0.0;
    if (has_forward_distance) {
      forward_pct = clamp_value(
          (forward_distance_mm - kAutoTargetRangeMm) * kAutoForwardGainPctPerMm,
          0.0,
          kAutoMaxForwardPct);
      if (std::fabs(lateral_error_norm) > kAutoCenterToleranceNorm) {
        forward_pct = std::min(forward_pct, kAutoCenteringForwardLimitPct);
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
    if (vision.last_detection.bbox_px.valid() && image_width_px > 0.0) {
      const double image_center_px = image_width_px * 0.5;
      const double bbox_center_px =
          vision.last_detection.bbox_px.x + vision.last_detection.bbox_px.width * 0.5;
      return clamp_abs((bbox_center_px - image_center_px) / image_center_px, 1.0);
    }

    if (vision.estimate_available && vision.last_estimate.valid &&
        basic::vision::is_finite(vision.last_estimate.ray_camera.x)) {
      return clamp_abs(vision.last_estimate.ray_camera.x, 1.0);
    }

    return 0.0;
  }

  double resolve_forward_distance_mm(const FootballVisionState& vision) const {
    if (!vision.estimate_available || !vision.last_estimate.valid) {
      return basic::vision::nan_value();
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

  void limit_drive_output() {
    const basic::chassis::HChassisState& state =
        basic::chassis::h_chassis_state(hardware_.football_chassis);
    const double max_abs = std::max(
        {std::fabs(state.left_pct), std::fabs(state.right_pct), std::fabs(state.center_pct)});
    if (max_abs <= kDriveOutputLimitPct || max_abs <= 0.0) {
      return;
    }

    const double scale = kDriveOutputLimitPct / max_abs;
    basic::chassis::h_drive_set_output(
        hardware_.football_chassis,
        state.left_pct * scale,
        state.right_pct * scale,
        state.center_pct * scale,
        state.stop_brake_type);
  }

  void apply_drive_request(
      double forward_pct,
      double strafe_pct,
      double turn_pct,
      vex::brakeType brake_type) {
    basic::chassis::h_drive_set_output(
        hardware_.football_chassis,
        forward_pct + turn_pct,
        forward_pct - turn_pct,
        strafe_pct,
        brake_type);
    limit_drive_output();
  }

  void stop_drive(vex::brakeType drive_brake_type) {
    basic::chassis::h_chassis_stop(hardware_.football_chassis, drive_brake_type);
  }

  void stop_drive_and_actuator_motion(vex::brakeType drive_brake_type) {
    stop_drive(drive_brake_type);
    auto& actuator_state =
        basic::mechanism::pneumatic_motor_actuator_state(hardware_.actuator);
    actuator_state.motor_pct = 0.0;
    actuator_state.motor_auto_active = false;
    hardware_.actuator.motor().stop(vex::coast);
    sync_actuator_state();
  }

  void set_actuator_pneumatic_open(bool open) {
    basic::mechanism::single_pneumatic_set_open(hardware_.actuator.pneumatic(), open);
    sync_actuator_state();
  }

  void sync_actuator_state() {
    basic::mechanism::pneumatic_motor_actuator_refresh_state(hardware_.actuator);
    state_.actuator = basic::mechanism::pneumatic_motor_actuator_state(hardware_.actuator);
  }

  void show_mode_status() {
    hardware_.controller.Screen.setCursor(1, 1);
    hardware_.controller.Screen.print(auto_mode_enabled_ ? "AUTO[Y]: ON  " : "AUTO[Y]: OFF ");
  }

  RobotHardware hardware_;
  RobotState state_;
  basic::vision::MonocularLocator locator_;
  vex::competition* competition_{nullptr};
  bool auto_mode_enabled_{false};
  bool auto_pickup_triggered_{false};

  friend FootballRobot& current_football_robot();
};

FootballRobot& current_football_robot() {
  static FootballRobot robot;
  return robot;
}

}  // namespace

basic::app::Robot& get_robot() {
  return current_football_robot();
}

void configure_vision(const FootballVisionConfig& config) {
  current_football_robot().configure_vision(config);
}

basic::vision::EstimateResult submit_yolo_detection(const YoloDetection& detection) {
  return current_football_robot().submit_yolo_detection(detection);
}

void clear_yolo_detection() {
  current_football_robot().clear_yolo_detection();
}

FootballVisionState get_vision_state() {
  return current_football_robot().vision_state();
}

basic::mechanism::PneumaticMotorActuatorState get_actuator_state() {
  return current_football_robot().actuator_state();
}

basic::mechanism::PneumaticMotorActuatorState refresh_actuator_state() {
  return current_football_robot().refresh_actuator_state();
}

void set_actuator_motor_angle_a() {
  current_football_robot().set_actuator_motor_angle_state(
      basic::mechanism::MotorPositionState::kAngleA);
}

void set_actuator_motor_angle_b() {
  current_football_robot().set_actuator_motor_angle_state(
      basic::mechanism::MotorPositionState::kAngleB);
}

void toggle_actuator_motor_angle_state() {
  current_football_robot().toggle_actuator_motor_angle_state();
}

void update_actuator_motor_to_target() {
  current_football_robot().update_actuator_motor_to_target();
}

}  // namespace basic::hardware::football_robot
