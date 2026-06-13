#include "mechanism/pneumatic_motor_actuator.h"

#include "control/motor_control.h"

#include <cmath>

namespace basic::mechanism {

namespace {

using basic::control::stopcontrol;
using basic::control::velocitycontrol;

constexpr double kManualMotorSpeedPct = 25.0;

vex::motor make_motor(const basic::device::MotorConfig& config) {
  return vex::motor{config.port, config.gear_ratio, config.reversed};
}

double position_error_deg(double position_deg, double target_deg) {
  return position_deg - target_deg;
}

MotorPositionState resolve_motor_position_state(
    const PneumaticMotorActuator& mechanism,
    double position_deg) {
  const double tolerance_deg = mechanism.config().motor_state_tolerance_deg;
  if (std::fabs(position_error_deg(
          position_deg,
          mechanism.config().motor_angle_a_deg)) <= tolerance_deg) {
    return MotorPositionState::kAngleA;
  }
  if (std::fabs(position_error_deg(
          position_deg,
          mechanism.config().motor_angle_b_deg)) <= tolerance_deg) {
    return MotorPositionState::kAngleB;
  }
  return MotorPositionState::kUnknown;
}

double target_angle_deg(
    const PneumaticMotorActuator& mechanism,
    MotorPositionState target_state) {
  switch (target_state) {
    case MotorPositionState::kAngleB:
      return mechanism.config().motor_angle_b_deg;
    case MotorPositionState::kAngleA:
    case MotorPositionState::kUnknown:
    default:
      return mechanism.config().motor_angle_a_deg;
  }
}

void refresh_motor_state(PneumaticMotorActuator& mechanism) {
  mechanism.state().motor_position_deg = mechanism.motor().position(vex::deg);
  mechanism.state().motor_position_state =
      resolve_motor_position_state(mechanism, mechanism.state().motor_position_deg);
}

void apply_motor(PneumaticMotorActuator& mechanism) {
  if (mechanism.state().motor_pct != 0.0) {
    velocitycontrol(mechanism.motor(), mechanism.state().motor_pct, vex::pct);
  } else {
    stopcontrol(mechanism.motor(), vex::coast);
  }
}

}  // namespace

PneumaticMotorActuator::PneumaticMotorActuator(const PneumaticMotorActuatorConfig& config)
    : config_(config),
      pneumatic_(single_pneumatic_init(config.pneumatic)),
      motor_(make_motor(config.motor)) {
  state_.motor_target_deg = config_.motor_angle_a_deg;
  state_.motor_position_deg = motor_.position(vex::deg);
  state_.motor_position_state = resolve_motor_position_state(*this, state_.motor_position_deg);
}

const PneumaticMotorActuatorConfig& PneumaticMotorActuator::config() const { return config_; }

SinglePneumatic& PneumaticMotorActuator::pneumatic() { return pneumatic_; }

const SinglePneumatic& PneumaticMotorActuator::pneumatic() const { return pneumatic_; }

vex::motor& PneumaticMotorActuator::motor() { return motor_; }

const vex::motor& PneumaticMotorActuator::motor() const { return motor_; }

PneumaticMotorActuatorState& PneumaticMotorActuator::state() { return state_; }

const PneumaticMotorActuatorState& PneumaticMotorActuator::state() const { return state_; }

PneumaticMotorActuator pneumatic_motor_actuator_init(
    const PneumaticMotorActuatorConfig& config) {
  return PneumaticMotorActuator(config);
}

PneumaticMotorActuatorCommand pneumatic_motor_actuator_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input) {
  PneumaticMotorActuatorCommand command;
  command.pneumatic = single_pneumatic_command_from_controller(input);
  command.toggle_motor_target_state = input.press_x;

  if (input.l1 && !input.l2) {
    command.motor_pct = kManualMotorSpeedPct;
  } else if (input.l2 && !input.l1) {
    command.motor_pct = -kManualMotorSpeedPct;
  }

  return command;
}

void pneumatic_motor_actuator_update(
    PneumaticMotorActuator& mechanism,
    const PneumaticMotorActuatorCommand& command) {
  single_pneumatic_update(mechanism.pneumatic(), command.pneumatic);
  mechanism.state().pneumatic = single_pneumatic_state(mechanism.pneumatic());
  refresh_motor_state(mechanism);

  if (command.toggle_motor_target_state) {
    pneumatic_motor_actuator_toggle_motor_target_state(mechanism);
    pneumatic_motor_actuator_update_motor_target(mechanism);
    return;
  }

  if (command.motor_pct != 0.0) {
    mechanism.state().motor_pct = command.motor_pct;
    mechanism.state().motor_auto_active = false;
    apply_motor(mechanism);
    refresh_motor_state(mechanism);
    return;
  }

  if (mechanism.state().motor_auto_active) {
    pneumatic_motor_actuator_update_motor_target(mechanism);
    return;
  }

  mechanism.state().motor_pct = 0.0;
  apply_motor(mechanism);
  refresh_motor_state(mechanism);
}

void pneumatic_motor_actuator_set_motor_target_state(
    PneumaticMotorActuator& mechanism,
    MotorPositionState target_state) {
  if (target_state == MotorPositionState::kUnknown) {
    return;
  }
  mechanism.state().motor_target_state = target_state;
  mechanism.state().motor_target_deg = target_angle_deg(mechanism, target_state);
}

void pneumatic_motor_actuator_toggle_motor_target_state(PneumaticMotorActuator& mechanism) {
  refresh_motor_state(mechanism);
  if (mechanism.state().motor_target_state == MotorPositionState::kAngleB ||
      mechanism.state().motor_position_state == MotorPositionState::kAngleB) {
    pneumatic_motor_actuator_set_motor_target_state(mechanism, MotorPositionState::kAngleA);
    return;
  }

  pneumatic_motor_actuator_set_motor_target_state(mechanism, MotorPositionState::kAngleB);
}

void pneumatic_motor_actuator_refresh_state(PneumaticMotorActuator& mechanism) {
  mechanism.state().pneumatic = single_pneumatic_state(mechanism.pneumatic());
  refresh_motor_state(mechanism);
}

void pneumatic_motor_actuator_update_motor_target(PneumaticMotorActuator& mechanism) {
  refresh_motor_state(mechanism);
  mechanism.state().motor_auto_active = true;
  mechanism.state().motor_pct = 0.0;
  mechanism.motor().spinToPosition(
      mechanism.state().motor_target_deg,
      vex::deg,
      mechanism.config().motor_auto_speed_pct,
      vex::velocityUnits::pct,
      false);
  refresh_motor_state(mechanism);
}

double pneumatic_motor_actuator_motor_angle_a_deg(const PneumaticMotorActuator& mechanism) {
  return mechanism.config().motor_angle_a_deg;
}

double pneumatic_motor_actuator_motor_angle_b_deg(const PneumaticMotorActuator& mechanism) {
  return mechanism.config().motor_angle_b_deg;
}

void pneumatic_motor_actuator_stop(PneumaticMotorActuator& mechanism) {
  single_pneumatic_stop(mechanism.pneumatic());
  mechanism.state() = PneumaticMotorActuatorState{};
  mechanism.state().pneumatic = single_pneumatic_state(mechanism.pneumatic());
  mechanism.state().motor_target_state = MotorPositionState::kAngleA;
  mechanism.state().motor_target_deg = mechanism.config().motor_angle_a_deg;
  refresh_motor_state(mechanism);
  apply_motor(mechanism);
}

PneumaticMotorActuatorState& pneumatic_motor_actuator_state(
    PneumaticMotorActuator& mechanism) {
  return mechanism.state();
}

const PneumaticMotorActuatorState& pneumatic_motor_actuator_state(
    const PneumaticMotorActuator& mechanism) {
  return mechanism.state();
}

}  // namespace basic::mechanism
