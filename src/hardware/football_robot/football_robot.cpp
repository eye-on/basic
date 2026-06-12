#include "hardware/robot_selector.h"

#include "chassis/h_chassis.h"
#include "chassis/h_drive.h"
#include "hardware/football_robot/robot_hardware.h"
#include "hardware/football_robot/robot_state.h"
#include "hardware/football_robot/vision.h"
#include "input/controller.h"
#include "mechanism/single_pneumatic.h"

#include <algorithm>
#include <cmath>

namespace basic::hardware::football_robot {

namespace {

class FootballRobot;
FootballRobot& current_football_robot();

class FootballRobot final : public basic::app::Robot {
 public:
  void initialize() override {
    configure_vision(FootballVisionConfig{});
    hardware_.calibrate_inertial_sensor();
    hardware_.show_calibrated();
  }

  void bind_background_tasks() override {}

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

 private:
  static void start_driver_control_entry() {
    current_football_robot().run_driver_control_loop();
  }

  static void start_autonomous_entry() {
    current_football_robot().run_autonomous_routine();
  }

  void run_driver_control_loop() {
    while (should_run_driver_control()) {
      basic::input::controller_update(hardware_.brain, hardware_.controller, state_.controller);

      const basic::chassis::HChassisCommand command =
          basic::chassis::h_chassis_command_from_controller(
              state_.controller,
              basic::chassis::h_chassis_state(hardware_.football_chassis).stop_brake_type);
      basic::chassis::h_chassis_update(hardware_.football_chassis, command);
      basic::mechanism::single_pneumatic_update(
          hardware_.actuator,
          basic::mechanism::single_pneumatic_command_from_controller(state_.controller));
      state_.actuator = basic::mechanism::single_pneumatic_state(hardware_.actuator);
      limit_drive_output();

      vex::this_thread::sleep_for(kRefreshTime);
    }

    stop_all_outputs(vex::coast);
  }

  void run_autonomous_routine() {
    stop_all_outputs(vex::hold);
  }

  bool should_run_driver_control() const {
    return competition_ != nullptr && competition_->isEnabled() && competition_->isDriverControl();
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

  void stop_all_outputs(vex::brakeType drive_brake_type) {
    state_.controller = basic::hardware::shared::ControllerInputState{};
    basic::chassis::h_chassis_stop(hardware_.football_chassis, drive_brake_type);
    basic::mechanism::single_pneumatic_stop(hardware_.actuator);
    state_.actuator = basic::mechanism::single_pneumatic_state(hardware_.actuator);
  }

  RobotHardware hardware_;
  RobotState state_;
  basic::vision::MonocularLocator locator_;
  vex::competition* competition_{nullptr};

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

}  // namespace basic::hardware::football_robot
