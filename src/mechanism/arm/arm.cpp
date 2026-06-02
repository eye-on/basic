#include "mechanism/arm/arm.h"

#include "control/motor_control.h"

namespace basic::mechanism::arm {

namespace {

using basic::control::stopcontrol;

vex::motor make_motor(const basic::device::MotorConfig& config) {
  return vex::motor{config.port, config.gear_ratio, config.reversed};
}

void spin_to_angle(
    vex::motor& motor,
    double target_angle,
    vex::rotationUnits angle_units,
    double speed,
    vex::velocityUnits speed_units) {
  motor.spinToPosition(target_angle, angle_units, speed, speed_units, false);
}

ArmJointAngles motor_positions_to_joint_angles(Arm& mechanism) {
  const auto& mapping = mechanism.config().ik_config.motor_mapping;

  ArmJointAngles motor_angles;
  motor_angles.q1 = mechanism.joint1_motor().position(mechanism.config().command_units);
  motor_angles.q2 = mechanism.joint2_motor().position(mechanism.config().command_units);
  motor_angles.q3 = mechanism.joint3_motor().position(mechanism.config().command_units);
  motor_angles.q4 = mechanism.joint4_motor().position(mechanism.config().command_units);

  ArmJointAngles joint_angles;
  joint_angles.q1 = (motor_angles.q1 - mapping[0].zero_offset) / mapping[0].direction;
  joint_angles.q2 = (motor_angles.q2 - mapping[1].zero_offset) / mapping[1].direction;
  joint_angles.q3 = (motor_angles.q3 - mapping[2].zero_offset) / mapping[2].direction;
  joint_angles.q4 = (motor_angles.q4 - mapping[3].zero_offset) / mapping[3].direction;
  return joint_angles;
}

}  // namespace

Arm::Arm(const ArmConfig& config)
    : config_(config),
      joint1_motor_(make_motor(config.joint1_motor)),
      joint2_motor_(make_motor(config.joint2_motor)),
      joint3_motor_(make_motor(config.joint3_motor)),
      joint4_motor_(make_motor(config.joint4_motor)) {}

vex::motor& Arm::joint1_motor() { return joint1_motor_; }
vex::motor& Arm::joint2_motor() { return joint2_motor_; }
vex::motor& Arm::joint3_motor() { return joint3_motor_; }
vex::motor& Arm::joint4_motor() { return joint4_motor_; }

const vex::motor& Arm::joint1_motor() const { return joint1_motor_; }
const vex::motor& Arm::joint2_motor() const { return joint2_motor_; }
const vex::motor& Arm::joint3_motor() const { return joint3_motor_; }
const vex::motor& Arm::joint4_motor() const { return joint4_motor_; }

const ArmConfig& Arm::config() const { return config_; }

ArmState& Arm::state() { return state_; }
const ArmState& Arm::state() const { return state_; }

Arm arm_init(const ArmConfig& config) {
  return Arm(config);
}

void arm_update(Arm& mechanism, const ArmCommand& command) {
  mechanism.state().last_command = command;
  if (!command.enabled) {
    arm_stop(mechanism);
    return;
  }

  ArmJointAngles previous_joint_angles = motor_positions_to_joint_angles(mechanism);
  const double q4_reference = command.hold_q4 ? previous_joint_angles.q4 : command.q4_reference;
  ArmIkSolution solution = arm_inverse_kinematics_solve(
      command.target,
      mechanism.config().ik_config,
      previous_joint_angles,
      q4_reference);
  mechanism.state().last_solution = solution;

  if (solution.status == ArmIkStatus::kUnreachable ||
      solution.status == ArmIkStatus::kNoValidSolution ||
      solution.status == ArmIkStatus::kJointLimitViolation) {
    arm_stop(mechanism);
    return;
  }

  spin_to_angle(
      mechanism.joint1_motor(),
      solution.motor_angles.q1,
      mechanism.config().command_units,
      mechanism.config().move_speed,
      mechanism.config().move_speed_units);
  spin_to_angle(
      mechanism.joint2_motor(),
      solution.motor_angles.q2,
      mechanism.config().command_units,
      mechanism.config().move_speed,
      mechanism.config().move_speed_units);
  spin_to_angle(
      mechanism.joint3_motor(),
      solution.motor_angles.q3,
      mechanism.config().command_units,
      mechanism.config().move_speed,
      mechanism.config().move_speed_units);
  spin_to_angle(
      mechanism.joint4_motor(),
      solution.motor_angles.q4,
      mechanism.config().command_units,
      mechanism.config().move_speed,
      mechanism.config().move_speed_units);
}

void arm_stop(Arm& mechanism, vex::brakeType brake_type) {
  stopcontrol(mechanism.joint1_motor(), brake_type);
  stopcontrol(mechanism.joint2_motor(), brake_type);
  stopcontrol(mechanism.joint3_motor(), brake_type);
  stopcontrol(mechanism.joint4_motor(), brake_type);
}

ArmState& arm_state(Arm& mechanism) {
  return mechanism.state();
}

const ArmState& arm_state(const Arm& mechanism) {
  return mechanism.state();
}

}  // namespace basic::mechanism::arm
