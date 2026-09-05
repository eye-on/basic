#include "mechanism/arm_2dof.h"

#include "control/motor_control.h"

namespace basic::mechanism {

namespace {

using basic::control::stopcontrol;
using basic::control::velocitycontrol;

vex::motor make_motor(const basic::device::MotorConfig& config) {
  return vex::motor{config.port, config.gear_ratio, config.reversed};
}

void refresh_state(Arm2Dof& mechanism) {
  auto& s = mechanism.state();
  const auto& c = mechanism.config();
  s.mode = c.mode;
  s.joint1_position =
      mechanism.motor1().position(c.position_units);
  s.joint2_position =
      mechanism.motor2().position(c.position_units);
}

void apply_open_loop(Arm2Dof& mechanism, const Arm2DofCommand& command) {
  auto& s = mechanism.state();
  s.joint1_direction = command.joint1_direction;
  s.joint2_direction = command.joint2_direction;

  const double speed = mechanism.config().velocity_speed_pct;
  // 开环速度：电平方向 → 固定幅值；同键同按方向冲突时以下发方向为准（后按不叠加）
  if (command.joint1_direction != 0) {
    velocitycontrol(mechanism.motor1(),
                    command.joint1_direction * speed, vex::pct);
  } else {
    stopcontrol(mechanism.motor1(), vex::hold);  // 松键 = 保持当前姿态
  }
  if (command.joint2_direction != 0) {
    velocitycontrol(mechanism.motor2(),
                    command.joint2_direction * speed, vex::pct);
  } else {
    stopcontrol(mechanism.motor2(), vex::hold);
  }
}

void apply_closed_loop(Arm2Dof& mechanism) {
  const auto& c = mechanism.config();
  // 闭环位置：固件位置环（非阻塞，每周期重新下发目标角）
  mechanism.motor1().spinToPosition(
      mechanism.state().joint1_target, c.position_units,
      c.position_speed_pct, vex::velocityUnits::pct, false);
  mechanism.motor2().spinToPosition(
      mechanism.state().joint2_target, c.position_units,
      c.position_speed_pct, vex::velocityUnits::pct, false);
}

}  // namespace

Arm2Dof::Arm2Dof(const Arm2DofConfig& config)
    : config_(config),
      motor1_(make_motor(config.motor1)),
      motor2_(make_motor(config.motor2)) {
  state_.mode = config.mode;
}

vex::motor& Arm2Dof::motor1() { return motor1_; }
vex::motor& Arm2Dof::motor2() { return motor2_; }
const vex::motor& Arm2Dof::motor1() const { return motor1_; }
const vex::motor& Arm2Dof::motor2() const { return motor2_; }

Arm2DofConfig& Arm2Dof::config() { return config_; }
const Arm2DofConfig& Arm2Dof::config() const { return config_; }

Arm2DofState& Arm2Dof::state() { return state_; }
const Arm2DofState& Arm2Dof::state() const { return state_; }

Arm2Dof arm_2dof_init(const Arm2DofConfig& config) {
  return Arm2Dof(config);
}

Arm2DofCommand arm_2dof_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input) {
  Arm2DofCommand command;
  // 上/下 = 关节 1；X/B = 关节 2（电平方向；同时按 上+下 视为停）
  if (input.up && !input.down) {
    command.joint1_direction = 1;
  } else if (input.down && !input.up) {
    command.joint1_direction = -1;
  }
  if (input.x && !input.b) {
    command.joint2_direction = 1;
  } else if (input.b && !input.x) {
    command.joint2_direction = -1;
  }
  return command;
}

void arm_2dof_update(Arm2Dof& mechanism, const Arm2DofCommand& command) {
  if (mechanism.config().mode == Arm2DofMode::kClosedLoopPosition) {
    apply_closed_loop(mechanism);
  } else {
    apply_open_loop(mechanism, command);
  }
  refresh_state(mechanism);
}

void arm_2dof_set_mode(Arm2Dof& mechanism, Arm2DofMode mode) {
  mechanism.config().mode = mode;
  mechanism.state().mode = mode;
}

void arm_2dof_set_joint1_target(Arm2Dof& mechanism, double target) {
  mechanism.state().joint1_target = target;
}

void arm_2dof_set_joint2_target(Arm2Dof& mechanism, double target) {
  mechanism.state().joint2_target = target;
}

void arm_2dof_stop(Arm2Dof& mechanism, vex::brakeType brake_type) {
  stopcontrol(mechanism.motor1(), brake_type);
  stopcontrol(mechanism.motor2(), brake_type);
}

Arm2DofState& arm_2dof_state(Arm2Dof& mechanism) {
  return mechanism.state();
}

const Arm2DofState& arm_2dof_state(const Arm2Dof& mechanism) {
  return mechanism.state();
}

}  // namespace basic::mechanism
