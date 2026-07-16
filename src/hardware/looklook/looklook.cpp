#include "hardware/robot_selector.h"

#include "chassis/heading_hold.h"
#include "chassis/x_chassis.h"
#include "hardware/looklook/autonomous.h"
#include "hardware/looklook/robot_hardware.h"
#include "hardware/looklook/robot_state.h"
#include "input/controller.h"
#include "mechanism/gripper.h"
#include "mechanism/linear_lift.h"

namespace basic::hardware::looklook {

namespace {

class LooklookRobot;
LooklookRobot& current_looklook();

class LooklookRobot final : public basic::app::Robot {
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
    current_looklook().run_driver_control_loop();
  }

  static void start_autonomous_entry() {
    current_looklook().run_autonomous_routine();
  }

  void run_driver_control_loop() {
    while (should_run_driver_control()) {
      basic::input::controller_update(hardware_.brain, hardware_.controller, state_.controller);
      auto cmd = basic::chassis::x_chassis_command_from_controller(
          state_.controller,
          basic::chassis::x_chassis_state(hardware_.x_chassis).stop_brake_type);
      cmd.turn_correction_pct = basic::chassis::heading_hold_update(
          hardware_.heading_hold,
          cmd.turn_input_pct,
          hardware_.inertial.heading(vex::deg));
      basic::chassis::x_chassis_update(hardware_.x_chassis, cmd);
      basic::mechanism::linear_lift_update(
          hardware_.lift,
          basic::mechanism::linear_lift_command_from_controller(state_.controller));
      basic::mechanism::gripper_update(
          hardware_.gripper,
          basic::mechanism::gripper_command_from_controller(state_.controller));
      vex::this_thread::sleep_for(kRefreshTime);
    }

    stop_all_outputs(vex::coast);
  }

  void run_autonomous_routine() {
    if (competition_ == nullptr) {
      return;
    }

    stop_all_outputs(vex::hold);
    autonomous::run_routine(hardware_, state_, *competition_);
  }

  bool should_run_driver_control() const {
    return competition_ != nullptr && competition_->isEnabled() && competition_->isDriverControl();
  }

  void stop_all_outputs(vex::brakeType drive_brake_type) {
    state_.controller = basic::hardware::shared::ControllerInputState{};
    basic::chassis::x_chassis_stop(hardware_.x_chassis, drive_brake_type);
    basic::mechanism::linear_lift_stop(hardware_.lift, vex::hold);
    basic::mechanism::gripper_stop(hardware_.gripper, vex::hold);
  }

  RobotHardware hardware_;
  RobotState state_;
  vex::competition* competition_{nullptr};

  friend LooklookRobot& current_looklook();
};

LooklookRobot& current_looklook() {
  static LooklookRobot robot;
  return robot;
}

}  // namespace

basic::app::Robot& get_robot() {
  return current_looklook();
}

}  // namespace basic::hardware::looklook
