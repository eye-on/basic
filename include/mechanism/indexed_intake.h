#ifndef BASIC_INCLUDE_INDEXED_INTAKE_H_
#define BASIC_INCLUDE_INDEXED_INTAKE_H_

#include "device_config.h"
#include "hardware/shared/state_types.h"

namespace basic::mechanism {

enum class IndexedIntakeMode {
  kOff,
  kLegacyIntake,
  kUnderThrow,
  kMiddleThrow,
  kUpperThrow,
};

enum class OverhangState {
  kCollapsed,
  kExpanded,
};

struct IndexedIntakeConfig {
  basic::device::MotorConfig trans_motor1;
  basic::device::MotorConfig trans_motor2;
  basic::device::MotorConfig trans_motor3;
  basic::device::MotorConfig trans_motor4;
  basic::device::MotorConfig under_motor1;
  basic::device::MotorConfig middle_motor1;
  basic::device::MotorConfig upper_motor1;
  basic::device::MotorConfig under_overhang_motor;
  basic::device::MotorConfig middle_overhang_motor;
  basic::device::MotorConfig upper_overhang_motor;
};

struct IndexedIntakeCommand {
  bool toggle_legacy_intake{false};
  bool toggle_under_throw{false};
  bool toggle_middle_throw{false};
  bool toggle_upper_throw{false};
  int under_overhang_pct{0};
  int middle_overhang_pct{0};
  int upper_overhang_pct{0};
};

struct IndexedIntakeState {
  IndexedIntakeMode mode{IndexedIntakeMode::kOff};
  OverhangState under_overhang{OverhangState::kCollapsed};
  OverhangState middle_overhang{OverhangState::kCollapsed};
  OverhangState upper_overhang{OverhangState::kCollapsed};
};

class IndexedIntake {
 public:
  explicit IndexedIntake(const IndexedIntakeConfig& config);

  vex::motor& trans_motor1();
  vex::motor& trans_motor2();
  vex::motor& trans_motor3();
  vex::motor& trans_motor4();
  vex::motor& under_motor1();
  vex::motor& middle_motor1();
  vex::motor& upper_motor1();
  vex::motor& under_overhang_motor();
  vex::motor& middle_overhang_motor();
  vex::motor& upper_overhang_motor();

  const vex::motor& trans_motor1() const;
  const vex::motor& trans_motor2() const;
  const vex::motor& trans_motor3() const;
  const vex::motor& trans_motor4() const;
  const vex::motor& under_motor1() const;
  const vex::motor& middle_motor1() const;
  const vex::motor& upper_motor1() const;
  const vex::motor& under_overhang_motor() const;
  const vex::motor& middle_overhang_motor() const;
  const vex::motor& upper_overhang_motor() const;

  IndexedIntakeState& state();
  const IndexedIntakeState& state() const;

 private:
  vex::motor trans_motor1_;
  vex::motor trans_motor2_;
  vex::motor trans_motor3_;
  vex::motor trans_motor4_;
  vex::motor under_motor1_;
  vex::motor middle_motor1_;
  vex::motor upper_motor1_;
  vex::motor under_overhang_motor_;
  vex::motor middle_overhang_motor_;
  vex::motor upper_overhang_motor_;
  IndexedIntakeState state_;
};

IndexedIntake indexed_intake_init(const IndexedIntakeConfig& config);
IndexedIntakeCommand indexed_intake_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input);

void indexed_intake_update(IndexedIntake& mechanism, const IndexedIntakeCommand& command);
void indexed_intake_toggle_mode(IndexedIntake& mechanism, IndexedIntakeMode requested_mode);
void indexed_intake_apply_mode(IndexedIntake& mechanism);
void indexed_intake_stop(
    IndexedIntake& mechanism,
    vex::brakeType indexed_brake_type = vex::coast,
    vex::brakeType overhang_brake_type = vex::hold);

void indexed_intake_toggle_under_overhang(IndexedIntake& mechanism);
void indexed_intake_toggle_middle_overhang(IndexedIntake& mechanism);
void indexed_intake_toggle_upper_overhang(IndexedIntake& mechanism);

IndexedIntakeState& indexed_intake_state(IndexedIntake& mechanism);
const IndexedIntakeState& indexed_intake_state(const IndexedIntake& mechanism);

}  // namespace basic::mechanism

#endif
