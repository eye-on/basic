#ifndef BASIC_INCLUDE_MECHANISM_ARM_INVERSE_KINEMATICS_H_
#define BASIC_INCLUDE_MECHANISM_ARM_INVERSE_KINEMATICS_H_

#include <array>

namespace basic::mechanism::arm {

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
  double min{-3.14159265358979323846};
  double max{3.14159265358979323846};
};

struct ArmMotorMapping {
  double direction{1.0};
  double zero_offset{0.0};
};

struct ArmIkConfig {
  double l1{0.0};
  double l2e{0.0};
  double rho_epsilon{1e-6};
  bool clamp_unreachable_target{false};
  std::array<ArmJointLimit, 4> joint_limits{};
  std::array<double, 4> continuity_weights{{1.0, 1.0, 1.0, 0.0}};
  std::array<ArmMotorMapping, 4> motor_mapping{};
};

struct ArmIkSolution {
  ArmIkStatus status{ArmIkStatus::kNoValidSolution};
  ArmElbowBranch branch{ArmElbowBranch::kPositive};
  bool reachable{false};
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
