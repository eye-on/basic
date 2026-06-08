#include "mechanism/arm/arm.h"

#include <algorithm>

#include "control/gearbox/software_gearbox.hpp"
#include "control/motor_control.h"

namespace basic::mechanism::arm {

namespace {

using basic::control::stopcontrol;

basic::control::SoftwareGearbox make_joint_gearbox(const ArmMotorMapping& mapping) {
  return basic::control::SoftwareGearbox({360.0, 360.0, mapping.gearbox_ratio, 0.0});
}

ArmMotorPositions read_motor_positions(Arm& mechanism) {
  ArmMotorPositions motor_positions;
  motor_positions.m1 = mechanism.joint1_motor().position(mechanism.config().command_units);
  motor_positions.m2 = mechanism.joint2_motor().position(mechanism.config().command_units);
  motor_positions.m3 = mechanism.joint3_motor().position(mechanism.config().command_units);
  motor_positions.m4 = mechanism.joint4_motor().position(mechanism.config().command_units);
  return motor_positions;
}

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
  const ArmMotorPositions motor_angles = read_motor_positions(mechanism);
  mechanism.state().last_motor_positions = motor_angles;

  const auto q1_gearbox = make_joint_gearbox(mapping[0]);
  const auto q2_gearbox = make_joint_gearbox(mapping[1]);
  const auto q3_gearbox = make_joint_gearbox(mapping[2]);
  const auto q4_gearbox = make_joint_gearbox(mapping[3]);

  ArmJointAngles joint_angles;
  joint_angles.q1 = q1_gearbox.output_from_input(
      (motor_angles.m1 - mapping[0].zero_offset) / (mapping[0].direction * mapping[0].units_per_radian));
  joint_angles.q2 = q2_gearbox.output_from_input(
      (motor_angles.m2 - mapping[1].zero_offset) / (mapping[1].direction * mapping[1].units_per_radian));
  joint_angles.q3 = q3_gearbox.output_from_input(
      (motor_angles.m3 - mapping[2].zero_offset) / (mapping[2].direction * mapping[2].units_per_radian));
  joint_angles.q4 = q4_gearbox.output_from_input(
      (motor_angles.m4 - mapping[3].zero_offset) / (mapping[3].direction * mapping[3].units_per_radian));
  mechanism.state().last_joint_angles = joint_angles;
  return joint_angles;
}

void update_calibration_state(Arm& mechanism) {
  mechanism.state().last_motor_positions = read_motor_positions(mechanism);

  const int interval = std::max(1, mechanism.config().calibration_print_interval_updates);
  mechanism.state().calibration_updates_since_print += 1;
  if (mechanism.state().calibration_updates_since_print < interval) {
    return;
  }

  mechanism.state().calibration_updates_since_print = 0;
  const auto& motor_positions = mechanism.state().last_motor_positions;
  printf(
      "arm calib: m1=%.2f, m2=%.2f, m3=%.2f, m4=%.2f\n",
      motor_positions.m1,
      motor_positions.m2,
      motor_positions.m3,
      motor_positions.m4);
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

ArmConfig& Arm::config() { return config_; }
const ArmConfig& Arm::config() const { return config_; }

ArmState& Arm::state() { return state_; }
const ArmState& Arm::state() const { return state_; }

Arm arm_init(const ArmConfig& config) {
  return Arm(config);
}

void arm_update(Arm& mechanism, const ArmCommand& command) {
  mechanism.state().last_command = command;
  if (mechanism.config().mode == ArmMode::kCalibration) {
    update_calibration_state(mechanism);
    arm_stop(mechanism, vex::coast);
    return;
  }

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
