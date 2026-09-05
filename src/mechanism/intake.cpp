#include "mechanism/intake.h"

#include "control/motor_control.h"

namespace basic::mechanism {

namespace {

using basic::control::stopcontrol;
using basic::control::velocitycontrol;

vex::motor make_motor(const basic::device::MotorConfig& config) {
  return vex::motor{config.port, config.gear_ratio, config.reversed};
}

void apply_mode(Intake& mechanism) {
  const auto& c = mechanism.config();
  const auto& s = mechanism.state();
  if (s.mode == IntakeMode::kRunning) {
    // 开环：直接下发速度百分比，两个电机同速同向
    velocitycontrol(mechanism.motor_a(), s.speed_pct, vex::pct);
    velocitycontrol(mechanism.motor_b(), s.speed_pct, vex::pct);
  } else {
    stopcontrol(mechanism.motor_a(), vex::coast);
    stopcontrol(mechanism.motor_b(), vex::coast);
  }
}

}  // namespace

Intake::Intake(const IntakeConfig& config)
    : config_(config),
      motor_a_(make_motor(config.motor_a)),
      motor_b_(make_motor(config.motor_b)) {}

vex::motor& Intake::motor_a() { return motor_a_; }
vex::motor& Intake::motor_b() { return motor_b_; }
const vex::motor& Intake::motor_a() const { return motor_a_; }
const vex::motor& Intake::motor_b() const { return motor_b_; }

IntakeConfig& Intake::config() { return config_; }
const IntakeConfig& Intake::config() const { return config_; }

IntakeState& Intake::state() { return state_; }
const IntakeState& Intake::state() const { return state_; }

Intake intake_init(const IntakeConfig& config) {
  return Intake(config);
}

IntakeCommand intake_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input) {
  IntakeCommand command;
  command.toggle = input.press_l1;  // L1 按下沿 → 翻转 开/停
  return command;
}

void intake_update(Intake& mechanism, const IntakeCommand& command) {
  // 边沿触发：按一下翻转 开/停（长按不重复触发）
  if (command.toggle) {
    if (mechanism.state().mode == IntakeMode::kOff) {
      mechanism.state().mode = IntakeMode::kRunning;
      mechanism.state().speed_pct = mechanism.config().speed_pct;
    } else {
      mechanism.state().mode = IntakeMode::kOff;
      mechanism.state().speed_pct = 0.0;
    }
  }
  apply_mode(mechanism);
}

void intake_set_running(Intake& mechanism, bool running) {
  mechanism.state().mode =
      running ? IntakeMode::kRunning : IntakeMode::kOff;
  mechanism.state().speed_pct =
      running ? mechanism.config().speed_pct : 0.0;
  apply_mode(mechanism);
}

void intake_stop(Intake& mechanism, vex::brakeType brake_type) {
  mechanism.state() = IntakeState{};
  stopcontrol(mechanism.motor_a(), brake_type);
  stopcontrol(mechanism.motor_b(), brake_type);
}

IntakeState& intake_state(Intake& mechanism) {
  return mechanism.state();
}

const IntakeState& intake_state(const Intake& mechanism) {
  return mechanism.state();
}

}  // namespace basic::mechanism
