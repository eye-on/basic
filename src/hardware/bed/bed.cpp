#include "hardware/robot_selector.h"

#include "chassis/bed_chassis.h"
#include "hardware/bed/autonomous.h"
#include "hardware/bed/robot_hardware.h"
#include "hardware/bed/robot_state.h"
#include "input/controller.h"
#include "mechanism/arm_2dof.h"
#include "mechanism/intake.h"
#include "mechanism/pneumatic_gripper.h"

namespace basic::hardware::bed {

namespace {

class BedRobot;
BedRobot& current_bed();

class BedRobot final : public basic::app::Robot {
 public:
  void initialize() override {
    hardware_.show_ready();
  }

  void bind_background_tasks() override {}

  void bind_competition(vex::competition& competition) override {
    competition_ = &competition;
    competition.autonomous(start_autonomous_entry);
    competition.drivercontrol(start_driver_control_entry);
  }

 private:
  static void start_driver_control_entry() {
    current_bed().run_driver_control_loop();
  }

  static void start_autonomous_entry() {
    current_bed().run_autonomous_routine();
  }

  void run_driver_control_loop() {
    while (should_run_driver_control()) {
      basic::input::controller_update(hardware_.brain, hardware_.controller, state_.controller);
      basic::chassis::bed_chassis_update(
          hardware_.bed_chassis,
          basic::chassis::bed_chassis_command_from_controller(
              state_.controller,
              basic::chassis::bed_chassis_state(hardware_.bed_chassis).stop_brake_type));
      basic::mechanism::intake_update(
          hardware_.intake,
          basic::mechanism::intake_command_from_controller(state_.controller));
      basic::mechanism::pneumatic_gripper_update(
          hardware_.pneumatic_gripper,
          basic::mechanism::pneumatic_gripper_command_from_controller(
              state_.controller));
      basic::mechanism::arm_2dof_update(
          hardware_.arm_2dof,
          basic::mechanism::arm_2dof_command_from_controller(state_.controller));
      vex::this_thread::sleep_for(kRefreshTime);
    }

    stop_all_outputs(vex::coast);
  }

  void run_autonomous_routine() {
    if (competition_ == nullptr) {
      return;
    }

    basic::hardware::bed::autonomous::run_routine(hardware_, state_, *competition_);
    stop_all_outputs(vex::hold);
  }

  bool should_run_driver_control() const {
    return competition_ != nullptr && competition_->isEnabled() && competition_->isDriverControl();
  }

  void stop_all_outputs(vex::brakeType brake_type) {
    state_.controller = basic::hardware::shared::ControllerInputState{};
    basic::chassis::bed_chassis_stop(hardware_.bed_chassis, brake_type);
    basic::mechanism::intake_stop(hardware_.intake, vex::coast);
    basic::mechanism::pneumatic_gripper_stop(hardware_.pneumatic_gripper);
    basic::mechanism::arm_2dof_stop(hardware_.arm_2dof, vex::hold);
  }

  RobotHardware hardware_;
  RobotState state_;
  vex::competition* competition_{nullptr};

  friend BedRobot& current_bed();
};

BedRobot& current_bed() {
  static BedRobot robot;
  return robot;
}

}  // namespace

basic::app::Robot& get_robot() {
  return current_bed();
}

}  // namespace basic::hardware::bed
