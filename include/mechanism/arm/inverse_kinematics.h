#ifndef BASIC_INCLUDE_MECHANISM_ARM_INVERSE_KINEMATICS_H_
#define BASIC_INCLUDE_MECHANISM_ARM_INVERSE_KINEMATICS_H_

#include <array>

namespace basic::mechanism::arm {

constexpr double kArmPi = 3.14159265358979323846;

constexpr double arm_degrees_to_radians(double degrees) {
  return degrees * kArmPi / 180.0;
}

enum class ArmIkStatus {
  kSuccess,
  kSingularBaseAxis,
  kUnreachable,
  kJointLimitViolation,
  kNoValidSolution,
};

enum class ArmElbowBranch {
  kNegative = -1,
  kPositive = 1,
};

struct ArmPoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct ArmJointAngles {
  double q1{0.0};
  double q2{0.0};
  double q3{0.0};
  double q4{0.0};
};

struct ArmJointLimit {
  // Allowed joint interval in radians.
  // If min <= max, the valid set is [min, max].
  // If min > max, the valid set wraps across the -pi/pi seam.
  double min{-3.14159265358979323846};
  double max{3.14159265358979323846};
};

struct ArmMotorMapping {
  double direction{1.0};
  // Encoder/command units per radian of motor shaft motion before external gearbox reduction.
  double units_per_radian{180.0 / kArmPi};
  // Encoder reading in motor units that corresponds to geometric joint angle 0.
  double zero_offset{0.0};
  // Additional motor-to-joint gearbox reduction. ratio=3 means motor turns 3x per output turn.
  double gearbox_ratio{1.0};
};

double arm_calculate_zero_offset_from_angle(
    double joint_angle_degrees,
    double motor_position_raw,
    double direction = 1.0,
    double units_per_radian = 3600.0 / (2.0 * kArmPi),
    double gearbox_ratio = 1.0);

struct ArmIkConfig {
  double l1{0.0};
  double l2e{0.0};
  double rho_epsilon{1e-6};
  bool clamp_unreachable_target{false};
  // Set to -1 to solve a mirrored assembly in the same software frame.
  int coordinate_sign{1};
  // Order: q1 base yaw, q2 shoulder pitch, q3 elbow relative angle, q4 forearm roll.
  std::array<ArmJointLimit, 4> joint_limits{{
      ArmJointLimit{-kArmPi, kArmPi},
      ArmJointLimit{0.0, kArmPi},
      ArmJointLimit{arm_degrees_to_radians(29.0), -arm_degrees_to_radians(71.0)},
      ArmJointLimit{-kArmPi, kArmPi},
  }};
  std::array<double, 4> continuity_weights{{1.0, 1.0, 1.0, 0.0}};
  std::array<ArmMotorMapping, 4> motor_mapping{{
      ArmMotorMapping{1.0, 180.0 / kArmPi, 0.0, 3.0},
      ArmMotorMapping{1.0, 180.0 / kArmPi, 0.0, 3.0},
      ArmMotorMapping{1.0, 180.0 / kArmPi, 0.0, 1.0},
      ArmMotorMapping{1.0, 180.0 / kArmPi, 0.0, 1.0},
  }};
};

struct ArmIkSolution {
  ArmIkStatus status{ArmIkStatus::kNoValidSolution};
  ArmElbowBranch branch{ArmElbowBranch::kPositive};
  bool reachable{false};
  bool within_distance_workspace{false};
  bool within_joint_limits{false};
  bool clamped_target{false};
  bool used_previous_q1{false};
  ArmPoint target{};
  ArmPoint solved_target{};
  double rho{0.0};
  double distance{0.0};
  double cost{0.0};
  ArmJointAngles joint_angles{};
  ArmJointAngles motor_angles{};
};

ArmIkSolution arm_inverse_kinematics_solve(
    const ArmPoint& target,
    const ArmIkConfig& config,
    const ArmJointAngles& previous_joint_angles,
    double q4_reference);

ArmIkSolution arm_inverse_kinematics_solve(
    const ArmPoint& target,
    const ArmIkConfig& config,
    const ArmJointAngles& previous_joint_angles);

ArmJointAngles arm_inverse_kinematics_map_to_motor(
    const ArmJointAngles& geometric_angles,
    const std::array<ArmMotorMapping, 4>& motor_mapping);

bool arm_inverse_kinematics_is_within_limits(
    const ArmJointAngles& joint_angles,
    const std::array<ArmJointLimit, 4>& limits);

}  // namespace basic::mechanism::arm

#endif
