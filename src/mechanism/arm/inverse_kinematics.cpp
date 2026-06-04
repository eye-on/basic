#include "mechanism/arm/inverse_kinematics.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace basic::mechanism::arm {

namespace {

constexpr double kPi = kArmPi;
constexpr double kTwoPi = 2.0 * kPi;

double clamp_value(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(max_value, value));
}

double wrap_angle(double angle) {
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

bool is_angle_within_limit(double angle, const ArmJointLimit& limit) {
  const double wrapped_angle = wrap_angle(angle);
  const double wrapped_min = wrap_angle(limit.min);
  const double wrapped_max = wrap_angle(limit.max);

  if (wrapped_min <= wrapped_max) {
    return wrapped_angle >= wrapped_min && wrapped_angle <= wrapped_max;
  }

  return wrapped_angle >= wrapped_min || wrapped_angle <= wrapped_max;
}

double joint_cost(
    const ArmJointAngles& candidate,
    const ArmJointAngles& previous,
    const std::array<double, 4>& weights) {
  return weights[0] * std::pow(wrap_angle(candidate.q1 - previous.q1), 2.0) +
         weights[1] * std::pow(wrap_angle(candidate.q2 - previous.q2), 2.0) +
         weights[2] * std::pow(wrap_angle(candidate.q3 - previous.q3), 2.0) +
         weights[3] * std::pow(wrap_angle(candidate.q4 - previous.q4), 2.0);
}

ArmPoint clamp_target_to_workspace(const ArmPoint& target, double min_distance, double max_distance) {
  const double distance = std::sqrt(target.x * target.x + target.y * target.y + target.z * target.z);
  if (distance == 0.0) {
    return ArmPoint{max_distance, 0.0, 0.0};
  }

  const double clamped_distance = clamp_value(distance, min_distance, max_distance);
  const double scale = clamped_distance / distance;
  return ArmPoint{target.x * scale, target.y * scale, target.z * scale};
}

ArmPoint apply_coordinate_sign(const ArmPoint& target, int coordinate_sign) {
  const double sign = coordinate_sign < 0 ? -1.0 : 1.0;
  return ArmPoint{sign * target.x, sign * target.y, sign * target.z};
}

ArmIkSolution solve_branch(
    const ArmPoint& target,
    const ArmIkConfig& config,
    const ArmJointAngles& previous_joint_angles,
    double q4_reference,
    ArmElbowBranch branch) {
  ArmIkSolution solution;
  solution.target = target;
  solution.solved_target = target;
  solution.branch = branch;
  solution.reachable = true;
  solution.within_distance_workspace = true;
  solution.clamped_target = false;

  solution.rho = std::hypot(target.x, target.y);
  solution.distance = std::hypot(solution.rho, target.z);

  if (solution.rho < config.rho_epsilon) {
    solution.joint_angles.q1 = previous_joint_angles.q1;
    solution.used_previous_q1 = true;
    solution.status = ArmIkStatus::kSingularBaseAxis;
  } else {
    solution.joint_angles.q1 = std::atan2(target.y, target.x);
    solution.status = ArmIkStatus::kSuccess;
  }

  const double c3_unclamped =
      (solution.distance * solution.distance - config.l1 * config.l1 - config.l2e * config.l2e) /
      (2.0 * config.l1 * config.l2e);
  const double c3 = clamp_value(c3_unclamped, -1.0, 1.0);
  const double s3_sign = static_cast<int>(branch) >= 0 ? 1.0 : -1.0;
  const double s3 = s3_sign * std::sqrt(std::max(0.0, 1.0 - c3 * c3));

  solution.joint_angles.q3 = std::atan2(s3, c3);

  const double alpha = std::atan2(target.z, solution.rho);
  const double beta = std::atan2(config.l2e * s3, config.l1 + config.l2e * c3);
  solution.joint_angles.q2 = (kPi / 2.0) - alpha + beta;
  solution.joint_angles.q4 = q4_reference;
  solution.motor_angles =
      arm_inverse_kinematics_map_to_motor(solution.joint_angles, config.motor_mapping);
  solution.cost = joint_cost(solution.joint_angles, previous_joint_angles, config.continuity_weights);
  return solution;
}

}  // namespace

ArmIkSolution arm_inverse_kinematics_solve(
    const ArmPoint& target,
    const ArmIkConfig& config,
    const ArmJointAngles& previous_joint_angles,
    double q4_reference) {
  ArmIkSolution unreachable_solution;
  unreachable_solution.target = target;
  unreachable_solution.solved_target = target;

  if (config.l1 <= 0.0 || config.l2e <= 0.0) {
    unreachable_solution.status = ArmIkStatus::kUnreachable;
    return unreachable_solution;
  }

  const double min_distance = std::abs(config.l1 - config.l2e);
  const double max_distance = config.l1 + config.l2e;
  const ArmPoint signed_target = apply_coordinate_sign(target, config.coordinate_sign);
  const double target_distance =
      std::sqrt(signed_target.x * signed_target.x + signed_target.y * signed_target.y + signed_target.z * signed_target.z);

  ArmPoint solved_target = signed_target;
  const bool within_distance_workspace = target_distance >= min_distance && target_distance <= max_distance;
  if (!within_distance_workspace) {
    if (!config.clamp_unreachable_target) {
      unreachable_solution.status = ArmIkStatus::kUnreachable;
      return unreachable_solution;
    }
    solved_target = clamp_target_to_workspace(signed_target, min_distance, max_distance);
  }

  ArmIkSolution positive =
      solve_branch(solved_target, config, previous_joint_angles, q4_reference, ArmElbowBranch::kPositive);
  ArmIkSolution negative =
      solve_branch(solved_target, config, previous_joint_angles, q4_reference, ArmElbowBranch::kNegative);

  positive.target = target;
  positive.solved_target = solved_target;
  positive.reachable = within_distance_workspace;
  positive.within_distance_workspace = within_distance_workspace;
  positive.clamped_target = !within_distance_workspace;

  negative.target = target;
  negative.solved_target = solved_target;
  negative.reachable = within_distance_workspace;
  negative.within_distance_workspace = within_distance_workspace;
  negative.clamped_target = !within_distance_workspace;

  const bool positive_valid =
      arm_inverse_kinematics_is_within_limits(positive.joint_angles, config.joint_limits);
  const bool negative_valid =
      arm_inverse_kinematics_is_within_limits(negative.joint_angles, config.joint_limits);

  positive.within_joint_limits = positive_valid;
  negative.within_joint_limits = negative_valid;

  if (!positive_valid) {
    positive.status = ArmIkStatus::kJointLimitViolation;
  }
  if (!negative_valid) {
    negative.status = ArmIkStatus::kJointLimitViolation;
  }

  if (positive_valid && negative_valid) {
    ArmIkSolution selected = positive.cost <= negative.cost ? positive : negative;
    selected.reachable = selected.within_distance_workspace && selected.within_joint_limits;
    return selected;
  }
  if (positive_valid) {
    positive.reachable = positive.within_distance_workspace && positive.within_joint_limits;
    return positive;
  }
  if (negative_valid) {
    negative.reachable = negative.within_distance_workspace && negative.within_joint_limits;
    return negative;
  }

  ArmIkSolution invalid = positive.cost <= negative.cost ? positive : negative;
  invalid.status = ArmIkStatus::kNoValidSolution;
  invalid.reachable = false;
  return invalid;
}

ArmIkSolution arm_inverse_kinematics_solve(
    const ArmPoint& target,
    const ArmIkConfig& config,
    const ArmJointAngles& previous_joint_angles) {
  return arm_inverse_kinematics_solve(target, config, previous_joint_angles, previous_joint_angles.q4);
}

ArmJointAngles arm_inverse_kinematics_map_to_motor(
    const ArmJointAngles& geometric_angles,
    const std::array<ArmMotorMapping, 4>& motor_mapping) {
  ArmJointAngles motor_angles;
  motor_angles.q1 =
      motor_mapping[0].direction * motor_mapping[0].units_per_radian * geometric_angles.q1 + motor_mapping[0].zero_offset;
  motor_angles.q2 =
      motor_mapping[1].direction * motor_mapping[1].units_per_radian * geometric_angles.q2 + motor_mapping[1].zero_offset;
  motor_angles.q3 =
      motor_mapping[2].direction * motor_mapping[2].units_per_radian * geometric_angles.q3 + motor_mapping[2].zero_offset;
  motor_angles.q4 =
      motor_mapping[3].direction * motor_mapping[3].units_per_radian * geometric_angles.q4 + motor_mapping[3].zero_offset;
  return motor_angles;
}

bool arm_inverse_kinematics_is_within_limits(
    const ArmJointAngles& joint_angles,
    const std::array<ArmJointLimit, 4>& limits) {
  return is_angle_within_limit(joint_angles.q1, limits[0]) &&
         is_angle_within_limit(joint_angles.q2, limits[1]) &&
         is_angle_within_limit(joint_angles.q3, limits[2]) &&
         is_angle_within_limit(joint_angles.q4, limits[3]);
}

}  // namespace basic::mechanism::arm
