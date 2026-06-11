#include "hardware/robot_selector.h"

#include "chassis/h_drive.h"
#include "chassis/new_chassis.h"
#include "hardware/football_robot/robot_hardware.h"
#include "hardware/football_robot/robot_state.h"
#include "input/controller.h"

#include <algorithm>
#include <cmath>

namespace basic::hardware::football_robot {

namespace {

class FootballRobot;
FootballRobot& current_football_robot();

class FootballRobot final : public basic::app::Robot {
 public:
  void initialize() override {
    hardware_.calibrate_inertial_sensor();
    hardware_.show_calibrated();
  }

  void bind_background_tasks() override {}

  void bind_competition(vex::competition& competition) override {
    competition_ = &competition;
    competition.autonomous(start_autonomous_entry);
    competition.drivercontrol(start_driver_control_entry);
  }

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

      const basic::chassis::NewChassisCommand command =
          basic::chassis::new_chassis_command_from_controller(
              state_.controller,
              basic::chassis::new_chassis_state(hardware_.football_chassis).stop_brake_type);
      basic::chassis::new_chassis_update(hardware_.football_chassis, command);
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
    const basic::chassis::NewChassisState& state =
        basic::chassis::new_chassis_state(hardware_.football_chassis);
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
    basic::chassis::new_chassis_stop(hardware_.football_chassis, drive_brake_type);
  }

  RobotHardware hardware_;
  RobotState state_;
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

}  // namespace basic::hardware::football_robot
