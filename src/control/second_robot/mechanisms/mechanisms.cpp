#include "control/second_robot/mechanisms.h"

#include "control/motor_control.h"

namespace basic::control::second_robot {

namespace {

using basic::control::stopcontrol;
using basic::control::velocitycontrol;
using basic::hardware::second_robot::RobotHardware;
using basic::hardware::second_robot::RobotState;
using basic::hardware::second_robot::ShooterMode;

void apply_shooter_mode(RobotHardware& hardware, ShooterMode mode, double speed_pct) {
  switch (mode) {
    case ShooterMode::kRoller:
      velocitycontrol(hardware.roller_lower_motor, speed_pct, vex::pct);
      velocitycontrol(hardware.roller_middle_motor, speed_pct, vex::pct);
      stopcontrol(hardware.roller_upper_motor, vex::coast);
      return;
    case ShooterMode::kMiddleShot:
      velocitycontrol(hardware.roller_lower_motor, speed_pct, vex::pct);
      velocitycontrol(hardware.roller_middle_motor, speed_pct, vex::pct);
      velocitycontrol(hardware.roller_upper_motor, -speed_pct, vex::pct);
      return;
    case ShooterMode::kLongShot:
      velocitycontrol(hardware.roller_lower_motor, speed_pct, vex::pct);
      velocitycontrol(hardware.roller_middle_motor, speed_pct, vex::pct);
      velocitycontrol(hardware.roller_upper_motor, speed_pct, vex::pct);
      return;
    case ShooterMode::kOff:
    default:
      stopcontrol(hardware.roller_lower_motor, vex::coast);
      stopcontrol(hardware.roller_middle_motor, vex::coast);
      stopcontrol(hardware.roller_upper_motor, vex::coast);
      return;
  }
}

void update_pneumatics(RobotHardware& hardware, const RobotState& state) {
  hardware.descore.set(state.mechanism.descore_open);
  hardware.hook.set(state.mechanism.hook_open);
  hardware.store.set(state.mechanism.store_open);
}

}  // namespace

void mechanism_update(
    RobotHardware& hardware,
    RobotState& state) {
  const basic::hardware::shared::ControllerInputState& input = state.controller;

  if (input.l1) {
    state.mechanism.shooter_mode = ShooterMode::kLongShot;
    state.mechanism.shooter_speed_pct = 80.0;
  } else if (input.l2) {
    state.mechanism.shooter_mode = ShooterMode::kLongShot;
    state.mechanism.shooter_speed_pct = -80.0;
  } else if (input.r2) {
    state.mechanism.shooter_mode = ShooterMode::kMiddleShot;
    state.mechanism.shooter_speed_pct = 80.0;
  } else if (input.r1) {
    state.mechanism.shooter_mode = ShooterMode::kLongShot;
    state.mechanism.shooter_speed_pct = 100.0;
  } else {
    state.mechanism.shooter_mode = ShooterMode::kOff;
    state.mechanism.shooter_speed_pct = 0.0;
  }

  if (input.press_up) {
    state.mechanism.descore_open = !state.mechanism.descore_open;
  }
  if (input.press_x) {
    state.mechanism.hook_open = !state.mechanism.hook_open;
  }
  if (input.press_y) {
    state.mechanism.store_open = !state.mechanism.store_open;
  }

  apply_shooter_mode(
      hardware,
      state.mechanism.shooter_mode,
      state.mechanism.shooter_speed_pct);
  update_pneumatics(hardware, state);
}

void set_shooter_mode(
    RobotHardware& hardware,
    RobotState& state,
    ShooterMode mode,
    double speed_pct) {
  state.mechanism.shooter_mode = mode;
  state.mechanism.shooter_speed_pct = speed_pct;
  apply_shooter_mode(hardware, mode, speed_pct);
}

}  // namespace basic::control::second_robot
