#include "mechanism/indexed_intake.h"

#include "control/motor_control.h"

#include <array>

namespace basic::mechanism {

namespace {

using basic::control::stopcontrol;
using basic::control::velocitycontrol;

constexpr std::size_t kIndexedMotorCount = 7;
constexpr std::array<int, kIndexedMotorCount> kOffSpeeds{{0, 0, 0, 0, 0, 0, 0}};
constexpr std::array<int, kIndexedMotorCount> kLegacyIntakeSpeeds{{0, 0, -100, 50, -100, 80, -70}};
constexpr std::array<int, kIndexedMotorCount> kUnderThrowSpeeds{{-100, 100, -100, 0, 100, 0, 0}};
constexpr std::array<int, kIndexedMotorCount> kMiddleThrowSpeeds{{-100, 100, -100, 100, -100, -100, 0}};
constexpr std::array<int, kIndexedMotorCount> kUpperThrowSpeeds{{-100, 100, -100, 100, -100, 70, 70}};

vex::motor make_motor(const basic::device::MotorConfig& config) {
  return vex::motor{config.port, config.gear_ratio, config.reversed};
}

void set_motor_power(vex::motor& motor, double speed_pct, vex::brakeType brake_type = vex::coast) {
  if (speed_pct != 0.0) {
    velocitycontrol(motor, speed_pct, vex::pct);
  } else {
    stopcontrol(motor, brake_type);
  }
}

void apply_overhang_power(vex::motor& motor, int speed_pct) {
  if (speed_pct != 0) {
    velocitycontrol(motor, speed_pct, vex::pct);
  } else {
    stopcontrol(motor, vex::hold);
  }
}

const std::array<int, kIndexedMotorCount>& indexed_motor_speeds(IndexedIntakeMode mode) {
  switch (mode) {
    case IndexedIntakeMode::kLegacyIntake:
      return kLegacyIntakeSpeeds;
    case IndexedIntakeMode::kUnderThrow:
      return kUnderThrowSpeeds;
    case IndexedIntakeMode::kMiddleThrow:
      return kMiddleThrowSpeeds;
    case IndexedIntakeMode::kUpperThrow:
      return kUpperThrowSpeeds;
    case IndexedIntakeMode::kOff:
    default:
      return kOffSpeeds;
  }
}

void apply_overhang_rotation(
    vex::motor& motor,
    OverhangState& state,
    double expand_degrees,
    double collapse_degrees,
    double speed_pct) {
  if (state == OverhangState::kExpanded) {
    state = OverhangState::kCollapsed;
    motor.spinFor(collapse_degrees, vex::deg, speed_pct, vex::velocityUnits::pct);
  } else {
    state = OverhangState::kExpanded;
    motor.spinFor(expand_degrees, vex::deg, speed_pct, vex::velocityUnits::pct);
  }
  motor.stop(vex::hold);
}

}  // namespace

IndexedIntake::IndexedIntake(const IndexedIntakeConfig& config)
    : trans_motor1_(make_motor(config.trans_motor1)),
      trans_motor2_(make_motor(config.trans_motor2)),
      trans_motor3_(make_motor(config.trans_motor3)),
      trans_motor4_(make_motor(config.trans_motor4)),
      under_motor1_(make_motor(config.under_motor1)),
      middle_motor1_(make_motor(config.middle_motor1)),
      upper_motor1_(make_motor(config.upper_motor1)),
      under_overhang_motor_(make_motor(config.under_overhang_motor)),
      middle_overhang_motor_(make_motor(config.middle_overhang_motor)),
      upper_overhang_motor_(make_motor(config.upper_overhang_motor)) {}

vex::motor& IndexedIntake::trans_motor1() { return trans_motor1_; }
vex::motor& IndexedIntake::trans_motor2() { return trans_motor2_; }
vex::motor& IndexedIntake::trans_motor3() { return trans_motor3_; }
vex::motor& IndexedIntake::trans_motor4() { return trans_motor4_; }
vex::motor& IndexedIntake::under_motor1() { return under_motor1_; }
vex::motor& IndexedIntake::middle_motor1() { return middle_motor1_; }
vex::motor& IndexedIntake::upper_motor1() { return upper_motor1_; }
vex::motor& IndexedIntake::under_overhang_motor() { return under_overhang_motor_; }
vex::motor& IndexedIntake::middle_overhang_motor() { return middle_overhang_motor_; }
vex::motor& IndexedIntake::upper_overhang_motor() { return upper_overhang_motor_; }

const vex::motor& IndexedIntake::trans_motor1() const { return trans_motor1_; }
const vex::motor& IndexedIntake::trans_motor2() const { return trans_motor2_; }
const vex::motor& IndexedIntake::trans_motor3() const { return trans_motor3_; }
const vex::motor& IndexedIntake::trans_motor4() const { return trans_motor4_; }
const vex::motor& IndexedIntake::under_motor1() const { return under_motor1_; }
const vex::motor& IndexedIntake::middle_motor1() const { return middle_motor1_; }
const vex::motor& IndexedIntake::upper_motor1() const { return upper_motor1_; }
const vex::motor& IndexedIntake::under_overhang_motor() const { return under_overhang_motor_; }
const vex::motor& IndexedIntake::middle_overhang_motor() const { return middle_overhang_motor_; }
const vex::motor& IndexedIntake::upper_overhang_motor() const { return upper_overhang_motor_; }

IndexedIntakeState& IndexedIntake::state() { return state_; }
const IndexedIntakeState& IndexedIntake::state() const { return state_; }

IndexedIntake indexed_intake_init(const IndexedIntakeConfig& config) {
  return IndexedIntake(config);
}

IndexedIntakeCommand indexed_intake_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input) {
  IndexedIntakeCommand command;
  command.toggle_legacy_intake = input.press_a;
  command.toggle_under_throw = input.press_l2;
  command.toggle_middle_throw = input.press_l1;
  command.toggle_upper_throw = input.press_r1;
  command.under_overhang_pct = input.x ? -20 : (input.b ? 20 : 0);
  command.middle_overhang_pct = input.left ? 20 : (input.right ? -20 : 0);
  command.upper_overhang_pct = input.up ? -50 : (input.down ? 50 : 0);
  return command;
}

void indexed_intake_toggle_mode(IndexedIntake& mechanism, IndexedIntakeMode requested_mode) {
  if (mechanism.state().mode == requested_mode) {
    mechanism.state().mode = IndexedIntakeMode::kOff;
  } else {
    mechanism.state().mode = requested_mode;
  }
}

void indexed_intake_apply_mode(IndexedIntake& mechanism) {
  const std::array<vex::motor*, kIndexedMotorCount> motors{{
      &mechanism.trans_motor1(),
      &mechanism.trans_motor2(),
      &mechanism.trans_motor3(),
      &mechanism.trans_motor4(),
      &mechanism.under_motor1(),
      &mechanism.middle_motor1(),
      &mechanism.upper_motor1(),
  }};
  const std::array<int, kIndexedMotorCount>& speeds = indexed_motor_speeds(mechanism.state().mode);
  for (std::size_t index = 0; index < kIndexedMotorCount; ++index) {
    set_motor_power(*motors[index], speeds[index]);
  }
}

void indexed_intake_update(IndexedIntake& mechanism, const IndexedIntakeCommand& command) {
  if (command.toggle_legacy_intake) {
    indexed_intake_toggle_mode(mechanism, IndexedIntakeMode::kLegacyIntake);
  }
  if (command.toggle_under_throw) {
    indexed_intake_toggle_mode(mechanism, IndexedIntakeMode::kUnderThrow);
  }
  if (command.toggle_middle_throw) {
    indexed_intake_toggle_mode(mechanism, IndexedIntakeMode::kMiddleThrow);
  }
  if (command.toggle_upper_throw) {
    indexed_intake_toggle_mode(mechanism, IndexedIntakeMode::kUpperThrow);
  }

  apply_overhang_power(mechanism.under_overhang_motor(), command.under_overhang_pct);
  apply_overhang_power(mechanism.middle_overhang_motor(), command.middle_overhang_pct);
  apply_overhang_power(mechanism.upper_overhang_motor(), command.upper_overhang_pct);
  indexed_intake_apply_mode(mechanism);
}

void indexed_intake_stop(
    IndexedIntake& mechanism,
    vex::brakeType indexed_brake_type,
    vex::brakeType overhang_brake_type) {
  mechanism.state() = IndexedIntakeState{};

  const std::array<vex::motor*, kIndexedMotorCount> indexed_motors{{
      &mechanism.trans_motor1(),
      &mechanism.trans_motor2(),
      &mechanism.trans_motor3(),
      &mechanism.trans_motor4(),
      &mechanism.under_motor1(),
      &mechanism.middle_motor1(),
      &mechanism.upper_motor1(),
  }};
  for (vex::motor* motor : indexed_motors) {
    stopcontrol(*motor, indexed_brake_type);
  }

  stopcontrol(mechanism.under_overhang_motor(), overhang_brake_type);
  stopcontrol(mechanism.middle_overhang_motor(), overhang_brake_type);
  stopcontrol(mechanism.upper_overhang_motor(), overhang_brake_type);
}

void indexed_intake_toggle_under_overhang(IndexedIntake& mechanism) {
  apply_overhang_rotation(
      mechanism.under_overhang_motor(),
      mechanism.state().under_overhang,
      750.0,
      -750.0,
      30.0);
}

void indexed_intake_toggle_middle_overhang(IndexedIntake& mechanism) {
  apply_overhang_rotation(
      mechanism.middle_overhang_motor(),
      mechanism.state().middle_overhang,
      550.0,
      -550.0,
      30.0);
}

void indexed_intake_toggle_upper_overhang(IndexedIntake& mechanism) {
  apply_overhang_rotation(
      mechanism.upper_overhang_motor(),
      mechanism.state().upper_overhang,
      -1500.0,
      1500.0,
      50.0);
}

IndexedIntakeState& indexed_intake_state(IndexedIntake& mechanism) {
  return mechanism.state();
}

const IndexedIntakeState& indexed_intake_state(const IndexedIntake& mechanism) {
  return mechanism.state();
}

}  // namespace basic::mechanism
