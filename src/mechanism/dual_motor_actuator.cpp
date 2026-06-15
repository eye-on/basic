#include "mechanism/dual_motor_actuator.h"

#include "control/motor_control.h"

#include <algorithm>
#include <cmath>

namespace basic::mechanism {

namespace {

using basic::control::stopcontrol;

vex::motor make_motor(const basic::device::MotorConfig& config) {
  return vex::motor{config.port, config.gear_ratio, config.reversed};
}

double clamp_speed_pct(double speed_pct) {
  return std::max(0.0, std::min(std::fabs(speed_pct), 100.0));
}

DualMotorActuatorPositionState resolve_secondary_position_state(
    const DualMotorActuator& mechanism,
    double position_deg) {
  const double tolerance_deg = mechanism.config().secondary_state_tolerance_deg;
  if (std::fabs(position_deg - mechanism.config().secondary_angle_a_deg) <= tolerance_deg) {
    return DualMotorActuatorPositionState::kAngleA;
  }
  if (std::fabs(position_deg - mechanism.config().secondary_angle_b_deg) <= tolerance_deg) {
    return DualMotorActuatorPositionState::kAngleB;
  }
  return DualMotorActuatorPositionState::kUnknown;
}

double target_secondary_angle_deg(
    const DualMotorActuator& mechanism,
    DualMotorActuatorPositionState target_state) {
  switch (target_state) {
    case DualMotorActuatorPositionState::kAngleB:
      return mechanism.config().secondary_angle_b_deg;
    case DualMotorActuatorPositionState::kAngleA:
    case DualMotorActuatorPositionState::kUnknown:
    default:
      return mechanism.config().secondary_angle_a_deg;
  }
}

void refresh_state(DualMotorActuator& mechanism) {
  mechanism.state().primary_motor_position_deg = mechanism.primary_motor().position(vex::deg);
  mechanism.state().secondary_motor_position_deg = mechanism.secondary_motor().position(vex::deg);
  mechanism.state().primary_motor_installed = mechanism.primary_motor().installed();
  mechanism.state().primary_motor_spinning = mechanism.primary_motor().isSpinning();
  mechanism.state().secondary_motor_installed = mechanism.secondary_motor().installed();
  mechanism.state().secondary_motor_spinning = mechanism.secondary_motor().isSpinning();
  mechanism.state().secondary_position_state = resolve_secondary_position_state(
      mechanism,
      mechanism.state().secondary_motor_position_deg);
}

void run_primary_stage(DualMotorActuator& mechanism, double stage_deg) {
  if (stage_deg == 0.0) {
    return;
  }

  mechanism.primary_motor().spinFor(
      stage_deg,
      vex::deg,
      clamp_speed_pct(mechanism.config().primary_speed_pct),
      vex::velocityUnits::pct);
}

void move_secondary_to_target(DualMotorActuator& mechanism) {
  mechanism.secondary_motor().spinToPosition(
      mechanism.state().secondary_target_deg,
      vex::deg,
      clamp_speed_pct(mechanism.config().secondary_speed_pct),
      vex::velocityUnits::pct,
      true);
  stopcontrol(mechanism.secondary_motor(), vex::hold);
}

}  // namespace

DualMotorActuator::DualMotorActuator(const DualMotorActuatorConfig& config)
    : config_(config),
      primary_motor_(make_motor(config.primary_motor)),
      secondary_motor_(make_motor(config.secondary_motor)) {
  state_.secondary_target_state = DualMotorActuatorPositionState::kAngleA;
  state_.secondary_target_deg = config_.secondary_angle_a_deg;
  refresh_state(*this);
}

const DualMotorActuatorConfig& DualMotorActuator::config() const { return config_; }

vex::motor& DualMotorActuator::primary_motor() { return primary_motor_; }

const vex::motor& DualMotorActuator::primary_motor() const { return primary_motor_; }

vex::motor& DualMotorActuator::secondary_motor() { return secondary_motor_; }

const vex::motor& DualMotorActuator::secondary_motor() const { return secondary_motor_; }

DualMotorActuatorState& DualMotorActuator::state() { return state_; }

const DualMotorActuatorState& DualMotorActuator::state() const { return state_; }

DualMotorActuator dual_motor_actuator_init(const DualMotorActuatorConfig& config) {
  return DualMotorActuator(config);
}

void dual_motor_actuator_update(
    DualMotorActuator& mechanism,
    const DualMotorActuatorCommand& command) {
  refresh_state(mechanism);

  if (command.run_primary_sequence) {
    dual_motor_actuator_run_primary_sequence(mechanism);
  }
  if (command.toggle_secondary_target_state) {
    dual_motor_actuator_toggle_secondary_target_state(mechanism);
    move_secondary_to_target(mechanism);
    refresh_state(mechanism);
  }
}

void dual_motor_actuator_run_primary_sequence(DualMotorActuator& mechanism) {
  refresh_state(mechanism);
  run_primary_stage(mechanism, mechanism.config().primary_stage1_deg);
  refresh_state(mechanism);
  run_primary_stage(mechanism, mechanism.config().primary_stage2_deg);
  stopcontrol(mechanism.primary_motor(), vex::hold);
  refresh_state(mechanism);
}

void dual_motor_actuator_set_secondary_target_state(
    DualMotorActuator& mechanism,
    DualMotorActuatorPositionState target_state) {
  if (target_state == DualMotorActuatorPositionState::kUnknown) {
    return;
  }

  mechanism.state().secondary_target_state = target_state;
  mechanism.state().secondary_target_deg = target_secondary_angle_deg(mechanism, target_state);
}

void dual_motor_actuator_toggle_secondary_target_state(DualMotorActuator& mechanism) {
  refresh_state(mechanism);
  if (mechanism.state().secondary_target_state == DualMotorActuatorPositionState::kAngleB ||
      mechanism.state().secondary_position_state == DualMotorActuatorPositionState::kAngleB) {
    dual_motor_actuator_set_secondary_target_state(
        mechanism,
        DualMotorActuatorPositionState::kAngleA);
    return;
  }

  dual_motor_actuator_set_secondary_target_state(
      mechanism,
      DualMotorActuatorPositionState::kAngleB);
}

void dual_motor_actuator_refresh_state(DualMotorActuator& mechanism) {
  refresh_state(mechanism);
}

void dual_motor_actuator_stop(DualMotorActuator& mechanism, vex::brakeType brake_type) {
  mechanism.state() = DualMotorActuatorState{};
  mechanism.state().secondary_target_state = DualMotorActuatorPositionState::kAngleA;
  mechanism.state().secondary_target_deg = mechanism.config().secondary_angle_a_deg;
  stopcontrol(mechanism.primary_motor(), brake_type);
  stopcontrol(mechanism.secondary_motor(), brake_type);
  refresh_state(mechanism);
}

double dual_motor_actuator_primary_stage1_deg(const DualMotorActuator& mechanism) {
  return mechanism.config().primary_stage1_deg;
}

double dual_motor_actuator_primary_stage2_deg(const DualMotorActuator& mechanism) {
  return mechanism.config().primary_stage2_deg;
}

double dual_motor_actuator_secondary_angle_a_deg(const DualMotorActuator& mechanism) {
  return mechanism.config().secondary_angle_a_deg;
}

double dual_motor_actuator_secondary_angle_b_deg(const DualMotorActuator& mechanism) {
  return mechanism.config().secondary_angle_b_deg;
}

DualMotorActuatorState& dual_motor_actuator_state(DualMotorActuator& mechanism) {
  return mechanism.state();
}

const DualMotorActuatorState& dual_motor_actuator_state(const DualMotorActuator& mechanism) {
  return mechanism.state();
}

}  // namespace basic::mechanism
