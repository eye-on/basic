#ifndef BASIC_INCLUDE_MECHANISM_PNEUMATIC_MOTOR_ACTUATOR_H_
#define BASIC_INCLUDE_MECHANISM_PNEUMATIC_MOTOR_ACTUATOR_H_

#include "device_config.h"
#include "hardware/shared/state_types.h"
#include "mechanism/single_pneumatic.h"

namespace basic::mechanism {

enum class MotorPositionState {
  kUnknown,
  kAngleA,
  kAngleB,
};

struct PneumaticMotorActuatorConfig {
  SinglePneumaticConfig pneumatic;
  basic::device::MotorConfig motor;
  double motor_angle_a_deg{0.0};
  double motor_angle_b_deg{180.0};
  double motor_auto_speed_pct{100.0};
  double motor_state_tolerance_deg{5.0};
};

struct PneumaticMotorActuatorCommand {
  SinglePneumaticCommand pneumatic;
  bool toggle_motor_target_state{false};
  double motor_pct{0.0};
};

struct PneumaticMotorActuatorState {
  SinglePneumaticState pneumatic;
  double motor_pct{0.0};
  double motor_position_deg{0.0};
  double motor_target_deg{0.0};
  bool motor_auto_active{false};
  MotorPositionState motor_position_state{MotorPositionState::kUnknown};
  MotorPositionState motor_target_state{MotorPositionState::kAngleA};
};

class PneumaticMotorActuator {
 public:
  explicit PneumaticMotorActuator(const PneumaticMotorActuatorConfig& config);

  const PneumaticMotorActuatorConfig& config() const;

  SinglePneumatic& pneumatic();
  const SinglePneumatic& pneumatic() const;

  vex::motor& motor();
  const vex::motor& motor() const;

  PneumaticMotorActuatorState& state();
  const PneumaticMotorActuatorState& state() const;

 private:
  PneumaticMotorActuatorConfig config_;
  SinglePneumatic pneumatic_;
  vex::motor motor_;
  PneumaticMotorActuatorState state_;
};

PneumaticMotorActuator pneumatic_motor_actuator_init(
    const PneumaticMotorActuatorConfig& config);

PneumaticMotorActuatorCommand pneumatic_motor_actuator_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input);

void pneumatic_motor_actuator_update(
    PneumaticMotorActuator& mechanism,
    const PneumaticMotorActuatorCommand& command);

void pneumatic_motor_actuator_set_motor_target_state(
    PneumaticMotorActuator& mechanism,
    MotorPositionState target_state);
void pneumatic_motor_actuator_toggle_motor_target_state(PneumaticMotorActuator& mechanism);
void pneumatic_motor_actuator_refresh_state(PneumaticMotorActuator& mechanism);
void pneumatic_motor_actuator_update_motor_target(PneumaticMotorActuator& mechanism);
double pneumatic_motor_actuator_motor_angle_a_deg(const PneumaticMotorActuator& mechanism);
double pneumatic_motor_actuator_motor_angle_b_deg(const PneumaticMotorActuator& mechanism);

void pneumatic_motor_actuator_stop(PneumaticMotorActuator& mechanism);

PneumaticMotorActuatorState& pneumatic_motor_actuator_state(
    PneumaticMotorActuator& mechanism);
const PneumaticMotorActuatorState& pneumatic_motor_actuator_state(
    const PneumaticMotorActuator& mechanism);

}  // namespace basic::mechanism

#endif  // BASIC_INCLUDE_MECHANISM_PNEUMATIC_MOTOR_ACTUATOR_H_
