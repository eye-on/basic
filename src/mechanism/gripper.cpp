#include "mechanism/gripper.h"

#include <algorithm>

#include "control/motor_control.h"

namespace basic::mechanism {

namespace {

using basic::control::get_done;
using basic::control::get_position;
using basic::control::stopcontrol;

vex::motor make_motor(const basic::device::MotorConfig& config) {
  return vex::motor{config.port, config.gear_ratio, config.reversed};
}

void refresh_state(Gripper& mechanism) {
  auto& s = mechanism.state();
  const auto& c = mechanism.config();

  s.motor_position = get_position(mechanism.motor(), c.position_units);
  s.at_target = get_done(mechanism.motor());
}

}  // namespace

Gripper::Gripper(const GripperConfig& config)
    : config_(config),
      motor_(make_motor(config.motor.motor)) {}

vex::motor& Gripper::motor() { return motor_; }
const vex::motor& Gripper::motor() const { return motor_; }

GripperConfig& Gripper::config() { return config_; }
const GripperConfig& Gripper::config() const { return config_; }

GripperState& Gripper::state() { return state_; }
const GripperState& Gripper::state() const { return state_; }

Gripper gripper_init(const GripperConfig& config) {
  return Gripper(config);
}

GripperCommand gripper_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input) {
  GripperCommand command;
  command.toggle = input.press_r2;
  return command;
}

void gripper_update(Gripper& mechanism, const GripperCommand& command) {
  // 边缘触发：按一下切换松开/夹住
  if (command.toggle) {
    if (mechanism.state().mode == GripperMode::kOpen) {
      mechanism.state().mode = GripperMode::kClosed;
    } else {
      mechanism.state().mode = GripperMode::kOpen;
    }
  }

  const auto& c = mechanism.config();
  const auto& slot = c.motor;

  // 根据模式选择目标位置，并做软件限位 clamp
  double target = (mechanism.state().mode == GripperMode::kClosed)
                      ? slot.position_max
                      : slot.position_min;
  target = std::max(slot.position_min, std::min(target, slot.position_max));

  mechanism.motor().spinToPosition(
      target, c.position_units, c.speed_pct, vex::velocityUnits::pct, false);

  refresh_state(mechanism);
}

void gripper_stop(Gripper& mechanism, vex::brakeType brake_type) {
  mechanism.state() = GripperState{};
  stopcontrol(mechanism.motor(), brake_type);
}

GripperState& gripper_state(Gripper& mechanism) {
  return mechanism.state();
}

const GripperState& gripper_state(const Gripper& mechanism) {
  return mechanism.state();
}

}  // namespace basic::mechanism
