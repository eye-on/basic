#include "mechanism/roller_shooter.h"

#include "control/motor_control.h"

namespace basic::mechanism {

namespace {

using basic::control::stopcontrol;
using basic::control::velocitycontrol;

vex::motor make_motor(const basic::device::MotorConfig& config) {
  return vex::motor{config.port, config.gear_ratio, config.reversed};
}

void apply_shooter_mode(
    vex::motor& lower,
    vex::motor& middle,
    vex::motor& upper,
    RollerShooterMode mode,
    double speed_pct) {
  switch (mode) {
    case RollerShooterMode::kRoller:
      velocitycontrol(lower, speed_pct, vex::pct);
      velocitycontrol(middle, speed_pct, vex::pct);
      stopcontrol(upper, vex::coast);
      return;
    case RollerShooterMode::kMiddleShot:
      velocitycontrol(lower, speed_pct, vex::pct);
      velocitycontrol(middle, speed_pct, vex::pct);
      velocitycontrol(upper, -speed_pct, vex::pct);
      return;
    case RollerShooterMode::kLongShot:
      velocitycontrol(lower, speed_pct, vex::pct);
      velocitycontrol(middle, speed_pct, vex::pct);
      velocitycontrol(upper, speed_pct, vex::pct);
      return;
    case RollerShooterMode::kOff:
    default:
      stopcontrol(lower, vex::coast);
      stopcontrol(middle, vex::coast);
      stopcontrol(upper, vex::coast);
      return;
  }
}

void apply_pneumatics(RollerShooter& mechanism) {
  mechanism.descore().set(mechanism.state().descore_open);
  mechanism.hook().set(mechanism.state().hook_open);
  mechanism.store().set(mechanism.state().store_open);
}

}  // namespace

RollerShooter::RollerShooter(const RollerShooterConfig& config)
    : roller_lower_motor_(make_motor(config.roller_lower_motor)),
      roller_middle_motor_(make_motor(config.roller_middle_motor)),
      roller_upper_motor_(make_motor(config.roller_upper_motor)),
      descore_(config.descore.port),
      hook_(config.hook.port),
      store_(config.store.port) {}

vex::motor& RollerShooter::roller_lower_motor() { return roller_lower_motor_; }
vex::motor& RollerShooter::roller_middle_motor() { return roller_middle_motor_; }
vex::motor& RollerShooter::roller_upper_motor() { return roller_upper_motor_; }
vex::digital_out& RollerShooter::descore() { return descore_; }
vex::digital_out& RollerShooter::hook() { return hook_; }
vex::digital_out& RollerShooter::store() { return store_; }

const vex::motor& RollerShooter::roller_lower_motor() const { return roller_lower_motor_; }
const vex::motor& RollerShooter::roller_middle_motor() const { return roller_middle_motor_; }
const vex::motor& RollerShooter::roller_upper_motor() const { return roller_upper_motor_; }
const vex::digital_out& RollerShooter::descore() const { return descore_; }
const vex::digital_out& RollerShooter::hook() const { return hook_; }
const vex::digital_out& RollerShooter::store() const { return store_; }

RollerShooterState& RollerShooter::state() { return state_; }
const RollerShooterState& RollerShooter::state() const { return state_; }

RollerShooter roller_shooter_init(const RollerShooterConfig& config) {
  return RollerShooter(config);
}

RollerShooterCommand roller_shooter_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input) {
  RollerShooterCommand command;
  if (input.l1) {
    command.shooter_mode = RollerShooterMode::kLongShot;
    command.shooter_speed_pct = 80.0;
  } else if (input.l2) {
    command.shooter_mode = RollerShooterMode::kLongShot;
    command.shooter_speed_pct = -80.0;
  } else if (input.r2) {
    command.shooter_mode = RollerShooterMode::kMiddleShot;
    command.shooter_speed_pct = 80.0;
  } else if (input.r1) {
    command.shooter_mode = RollerShooterMode::kLongShot;
    command.shooter_speed_pct = 100.0;
  }

  command.toggle_descore = input.press_up;
  command.toggle_hook = input.press_x;
  command.toggle_store = input.press_y;
  return command;
}

void roller_shooter_set_mode(
    RollerShooter& mechanism,
    RollerShooterMode mode,
    double speed_pct) {
  mechanism.state().shooter_mode = mode;
  mechanism.state().shooter_speed_pct = speed_pct;
  apply_shooter_mode(
      mechanism.roller_lower_motor(),
      mechanism.roller_middle_motor(),
      mechanism.roller_upper_motor(),
      mechanism.state().shooter_mode,
      mechanism.state().shooter_speed_pct);
}

void roller_shooter_set_descore(RollerShooter& mechanism, bool open) {
  mechanism.state().descore_open = open;
  mechanism.descore().set(open);
}

void roller_shooter_set_hook(RollerShooter& mechanism, bool open) {
  mechanism.state().hook_open = open;
  mechanism.hook().set(open);
}

void roller_shooter_set_store(RollerShooter& mechanism, bool open) {
  mechanism.state().store_open = open;
  mechanism.store().set(open);
}

void roller_shooter_update(RollerShooter& mechanism, const RollerShooterCommand& command) {
  if (command.toggle_descore) {
    mechanism.state().descore_open = !mechanism.state().descore_open;
  }
  if (command.toggle_hook) {
    mechanism.state().hook_open = !mechanism.state().hook_open;
  }
  if (command.toggle_store) {
    mechanism.state().store_open = !mechanism.state().store_open;
  }

  roller_shooter_set_mode(mechanism, command.shooter_mode, command.shooter_speed_pct);
  apply_pneumatics(mechanism);
}

void roller_shooter_stop(RollerShooter& mechanism) {
  mechanism.state() = RollerShooterState{};
  roller_shooter_set_mode(mechanism, RollerShooterMode::kOff, 0.0);
  apply_pneumatics(mechanism);
}

RollerShooterState& roller_shooter_state(RollerShooter& mechanism) {
  return mechanism.state();
}

const RollerShooterState& roller_shooter_state(const RollerShooter& mechanism) {
  return mechanism.state();
}

}  // namespace basic::mechanism
