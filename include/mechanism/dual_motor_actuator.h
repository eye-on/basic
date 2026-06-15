#ifndef BASIC_INCLUDE_MECHANISM_DUAL_MOTOR_ACTUATOR_H_
#define BASIC_INCLUDE_MECHANISM_DUAL_MOTOR_ACTUATOR_H_

#include "device_config.h"
#include "vex.h"

namespace basic::mechanism {

enum class DualMotorActuatorPositionState {
  kUnknown,
  kAngleA,
  kAngleB,
};

struct DualMotorActuatorConfig {
  basic::device::MotorConfig primary_motor;
  basic::device::MotorConfig secondary_motor;

  // Relative motion of the primary motor in the two-stage sequence.
  double primary_stage1_deg{45.0};
  double primary_stage2_deg{315.0};
  double primary_speed_pct{30.0};

  // Absolute targets for the secondary motor in vex::deg.
  double secondary_angle_a_deg{0.0};
  double secondary_angle_b_deg{180.0};
  double secondary_speed_pct{30.0};
  double secondary_state_tolerance_deg{5.0};
};

struct DualMotorActuatorCommand {
  bool run_primary_sequence{false};
  bool toggle_secondary_target_state{false};
};

struct DualMotorActuatorState {
  double primary_motor_position_deg{0.0};
  double secondary_motor_position_deg{0.0};
  double secondary_target_deg{0.0};
  bool primary_motor_installed{false};
  bool primary_motor_spinning{false};
  bool secondary_motor_installed{false};
  bool secondary_motor_spinning{false};
  DualMotorActuatorPositionState secondary_position_state{
      DualMotorActuatorPositionState::kUnknown};
  DualMotorActuatorPositionState secondary_target_state{
      DualMotorActuatorPositionState::kAngleA};
};

class DualMotorActuator {
 public:
  explicit DualMotorActuator(const DualMotorActuatorConfig& config);

  const DualMotorActuatorConfig& config() const;

  vex::motor& primary_motor();
  const vex::motor& primary_motor() const;

  vex::motor& secondary_motor();
  const vex::motor& secondary_motor() const;

  DualMotorActuatorState& state();
  const DualMotorActuatorState& state() const;

 private:
  DualMotorActuatorConfig config_;
  vex::motor primary_motor_;
  vex::motor secondary_motor_;
  DualMotorActuatorState state_;
};

DualMotorActuator dual_motor_actuator_init(const DualMotorActuatorConfig& config);

void dual_motor_actuator_update(
    DualMotorActuator& mechanism,
    const DualMotorActuatorCommand& command);

void dual_motor_actuator_run_primary_sequence(DualMotorActuator& mechanism);
void dual_motor_actuator_set_secondary_target_state(
    DualMotorActuator& mechanism,
    DualMotorActuatorPositionState target_state);
void dual_motor_actuator_toggle_secondary_target_state(DualMotorActuator& mechanism);
void dual_motor_actuator_refresh_state(DualMotorActuator& mechanism);
void dual_motor_actuator_stop(
    DualMotorActuator& mechanism,
    vex::brakeType brake_type = vex::hold);

double dual_motor_actuator_primary_stage1_deg(const DualMotorActuator& mechanism);
double dual_motor_actuator_primary_stage2_deg(const DualMotorActuator& mechanism);
double dual_motor_actuator_secondary_angle_a_deg(const DualMotorActuator& mechanism);
double dual_motor_actuator_secondary_angle_b_deg(const DualMotorActuator& mechanism);

DualMotorActuatorState& dual_motor_actuator_state(DualMotorActuator& mechanism);
const DualMotorActuatorState& dual_motor_actuator_state(const DualMotorActuator& mechanism);

}  // namespace basic::mechanism

#endif  // BASIC_INCLUDE_MECHANISM_DUAL_MOTOR_ACTUATOR_H_
