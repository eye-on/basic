#ifndef BASIC_INCLUDE_MECHANISM_ARM_ARM_H_
#define BASIC_INCLUDE_MECHANISM_ARM_ARM_H_

#include "device_config.h"
#include "mechanism/arm/inverse_kinematics.h"

namespace basic::mechanism::arm {

enum class ArmMode {
  kOperate,
  kCalibration,
};

struct ArmMotorPositions {
  double m1{0.0};
  double m2{0.0};
  double m3{0.0};
  double m4{0.0};
};

struct ArmConfig {
  basic::device::MotorConfig joint1_motor;
  basic::device::MotorConfig joint2_motor;
  basic::device::MotorConfig joint3_motor;
  basic::device::MotorConfig joint4_motor;
  ArmIkConfig ik_config{};
  ArmMode mode{ArmMode::kOperate};
  // Print one calibration line every N arm_update() calls.
  int calibration_print_interval_updates{25};
  vex::rotationUnits command_units{vex::deg};
  vex::velocityUnits move_speed_units{vex::velocityUnits::pct};
  double move_speed{30.0};
};

struct ArmCommand {
  ArmPoint target{};
  double q4_reference{0.0};
  bool hold_q4{true};
  bool enabled{false};
};

struct ArmState {
  ArmIkSolution last_solution{};
  ArmCommand last_command{};
  ArmMotorPositions last_motor_positions{};
  ArmJointAngles last_joint_angles{};
  int calibration_updates_since_print{0};
};

class Arm {
 public:
  explicit Arm(const ArmConfig& config);

  vex::motor& joint1_motor();
  vex::motor& joint2_motor();
  vex::motor& joint3_motor();
  vex::motor& joint4_motor();

  const vex::motor& joint1_motor() const;
  const vex::motor& joint2_motor() const;
  const vex::motor& joint3_motor() const;
  const vex::motor& joint4_motor() const;

  ArmConfig& config();
  const ArmConfig& config() const;
  ArmState& state();
  const ArmState& state() const;

 private:
  ArmConfig config_;
  vex::motor joint1_motor_;
  vex::motor joint2_motor_;
  vex::motor joint3_motor_;
  vex::motor joint4_motor_;
  ArmState state_;
};

Arm arm_init(const ArmConfig& config);
void arm_update(Arm& mechanism, const ArmCommand& command);
void arm_stop(Arm& mechanism, vex::brakeType brake_type = vex::hold);

ArmState& arm_state(Arm& mechanism);
const ArmState& arm_state(const Arm& mechanism);

}  // namespace basic::mechanism::arm

#endif
