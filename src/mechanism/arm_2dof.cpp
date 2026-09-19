#include "mechanism/arm_2dof.h"

#include <cmath>

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
  // 关节 1 双电机取平均位置（组内同步由物理同轴保证）
  s.joint1_position = 0.5 * (
      mechanism.motor1_a().position(c.position_units) +
      mechanism.motor1_b().position(c.position_units));
  s.joint2_position =
      mechanism.motor2().position(c.position_units);
}

void apply_open_loop(Arm2Dof& mechanism, const Arm2DofCommand& command) {
  auto& s = mechanism.state();
  s.joint1_direction = command.joint1_direction;
  s.joint2_direction = command.joint2_direction;

  const double speed = mechanism.config().velocity_speed_pct;
  const vex::brakeType stop_mode = mechanism.config().stop_brake_type;
  // 开环速度：电平方向 → 固定幅值；同键同按方向冲突时以下发方向为准（后按不叠加）
  if (command.joint1_direction != 0) {
    velocitycontrol(mechanism.motor1_a(),
                    command.joint1_direction * speed, vex::pct);
    velocitycontrol(mechanism.motor1_b(),
                    command.joint1_direction * speed, vex::pct);
  } else {
    stopcontrol(mechanism.motor1_a(), stop_mode);  // 松键 = 按停止模式保持姿态
    stopcontrol(mechanism.motor1_b(), stop_mode);
  }
  if (command.joint2_direction != 0) {
    velocitycontrol(mechanism.motor2(),
                    command.joint2_direction * speed, vex::pct);
  } else {
    stopcontrol(mechanism.motor2(), stop_mode);
  }
}

/// 闭环模式：按键按下沿把目标切到 home / alt 预设位置
/// 联锁（config.joint12_interlock，默认开）：
///   1. 关节 1 未展开（不在 alt）→ 拒绝关节 2 展开
///   2. 关节 2 未收回（还在 alt）→ 拒绝关节 1 收回
/// 因此标准流程：展开 = 上(关节1) → X(关节2)；收回 = B(关节2) → 下(关节1)
void apply_position_edges(Arm2Dof& mechanism, const Arm2DofCommand& command) {
  const auto& c = mechanism.config();
  auto& s = mechanism.state();

  // 关节 1：展开随时允许；收回需关节 2 已收回
  if (command.joint1_to_alt) {
    s.joint1_target = c.joint1_alt;
    s.joint1_at_alt = true;
    s.joint1_timeout = false;  // 新指令 → 解除超时放弃
  } else if (command.joint1_to_home) {
    if (!c.joint12_interlock || !s.joint2_at_alt) {
      s.joint1_target = c.joint1_home;
      s.joint1_at_alt = false;
      s.joint1_retract_blocked = false;
      s.joint1_timeout = false;
    } else {
      s.joint1_retract_blocked = true;  // 关节 2 未收回，拒绝
    }
  }

  // 关节 2：收回随时允许；展开需关节 1 已展开
  if (command.joint2_to_alt) {
    if (!c.joint12_interlock || s.joint1_at_alt) {
      s.joint2_target = c.joint2_alt;
      s.joint2_at_alt = true;
      s.joint2_deploy_blocked = false;
      s.joint2_timeout = false;
    } else {
      s.joint2_deploy_blocked = true;  // 关节 1 未展开，拒绝
    }
  } else if (command.joint2_to_home) {
    s.joint2_target = c.joint2_home;
    s.joint2_at_alt = false;
    s.joint2_timeout = false;
  }
}

/// 开环 + 堵转找限位（单关节）：
///  - 期望状态 want_alt 与已知物理状态 reached_alt 不一致 → 定速朝该方向转
///  - 速度连续低于阈值 stall_confirm_ms → 判定顶到机械限位：停机、更新已知状态
///  - 起动宽限 stall_grace_ms 内不检测；单次行程超过 stall_timeout_ms 停机放弃
/// allowed=false（联锁）时仅保持不动
void run_joint_stall(
    Arm2Dof& mechanism,
    bool allowed,
    vex::motor& motor_a,
    vex::motor* motor_b,
    bool want_alt,
    int alt_sign,
    bool& reached_alt,
    bool& moving,
    bool& stalled,
    bool& timeout,
    int& stall_since_ms,
    int& move_start_ms) {
  const auto& c = mechanism.config();
  const int now = static_cast<int>(vex::timer::system());

  auto stop_both = [&]() {
    stopcontrol(motor_a, c.stop_brake_type);
    if (motor_b != nullptr) {
      stopcontrol(*motor_b, c.stop_brake_type);
    }
  };

  // 已在目标限位 / 联锁不允许 / 上次超时未复位 → 保持不动
  if (!allowed || want_alt == reached_alt || timeout) {
    stop_both();
    moving = false;
    stalled = false;
    stall_since_ms = 0;
    return;
  }

  if (!moving) {  // 本次行程起点
    moving = true;
    stalled = false;
    stall_since_ms = 0;
    move_start_ms = now;
  }

  const int elapsed = now - move_start_ms;

  const double dir = (want_alt ? alt_sign : -alt_sign) * c.stall_speed_pct;
  velocitycontrol(motor_a, dir, vex::pct);
  if (motor_b != nullptr) {
    velocitycontrol(*motor_b, dir, vex::pct);
  }

  if (elapsed < c.stall_grace_ms) {  // 起动宽限：电机尚未起速
    stall_since_ms = 0;
    return;
  }
  if (elapsed >= c.stall_timeout_ms) {  // 安全超时：停机放弃（需新指令重试）
    stop_both();
    moving = false;
    stalled = false;
    timeout = true;
    stall_since_ms = 0;
    return;
  }

  const double v_a = std::fabs(motor_a.velocity(vex::rpm));
  const double v_b =
      (motor_b != nullptr) ? std::fabs(motor_b->velocity(vex::rpm)) : 0.0;
  const bool slow_now =
      (v_a <= c.stall_velocity_rpm) && (motor_b == nullptr || v_b <= c.stall_velocity_rpm);

  if (!slow_now) {
    stall_since_ms = 0;
    return;
  }

  if (stall_since_ms == 0) {
    stall_since_ms = now;
    return;
  }
  if (now - stall_since_ms >= c.stall_confirm_ms) {  // 顶到机械限位
    stop_both();
    moving = false;
    stalled = true;
    stall_since_ms = 0;
    reached_alt = want_alt;  // 更新已知物理状态
  }
}

/// 单键整臂序列：Y 边沿 → 收缩 ↔ 展开（两关节按序动作，靠堵转到位推进）
///   展开：关节1 先顶到展开限位 → 关节2 再顶到展开限位 → arm_extended = true
///   收回：关节2 先顶到收回限位 → 关节1 再顶到收回限位 → arm_extended = false
///   行程超时会取消序列并锁定该关节；再按 Y 启动新序列时解除锁定并重试
void apply_arm_sequence(Arm2Dof& mechanism, const Arm2DofCommand& command) {
  auto& s = mechanism.state();

  // 行程超时 → 序列暂停（等新的 Y 指令重试）
  if (s.joint1_timeout || s.joint2_timeout) {
    s.arm_sequence_step = 0;
  }

  // 展开许可被撤销（外部联锁）→ 中止展开序列，回到收回意图
  if (s.arm_sequence_step == 1 && !s.extend_allowed) {
    s.arm_sequence_step = 0;
    s.joint1_at_alt = false;
    s.joint2_at_alt = false;
  }

  // 物理状态判定：任一关节在展开位 → 视为"未收回"，按 Y 执行收回
  const bool any_extended = s.joint1_alt_reached || s.joint2_alt_reached;

  // 按键边沿：启动新序列（同时解除两个关节的超时锁定，否则序列会卡住）
  if (command.arm_toggle) {
    s.joint1_timeout = false;
    s.joint2_timeout = false;
    if (any_extended) {
      s.arm_sequence_step = 2;  // 收回（始终允许）
      s.joint2_at_alt = false;
      s.joint1_at_alt = true;   // 关节 1 先保持展开
      s.extend_blocked = false;
    } else if (s.extend_allowed) {
      s.arm_sequence_step = 1;  // 展开
      s.joint1_at_alt = true;
      s.joint2_at_alt = false;  // 关节 2 先保持收回
      s.extend_blocked = false;
    } else {
      s.extend_blocked = true;  // 联锁拒绝展开（如框住机构处于展开态）
    }
  }

  // 序列推进（依据物理到位状态）
  switch (s.arm_sequence_step) {
    case 1:  // 正在展开
      if (!s.joint1_alt_reached) {
        s.joint1_at_alt = true;
        s.joint2_at_alt = false;
      } else if (!s.joint2_alt_reached) {
        s.joint1_at_alt = true;
        s.joint2_at_alt = true;
      } else {
        s.arm_extended = true;
        s.arm_sequence_step = 0;
      }
      break;

    case 2:  // 正在收回
      if (s.joint2_alt_reached) {
        s.joint1_at_alt = true;
        s.joint2_at_alt = false;
      } else if (s.joint1_alt_reached) {
        s.joint1_at_alt = false;
        s.joint2_at_alt = false;
      } else {
        s.arm_extended = false;
        s.arm_sequence_step = 0;
      }
      break;

    default:
      break;
  }
}

/// 堵转找限位：驱动两个关节到期望限位；联锁按"物理到达"判定
///   关节 2 展开 需 关节 1 已顶到 alt 限位
///   关节 1 收回 需 关节 2 已离开 alt 限位
void apply_stall_limit(Arm2Dof& mechanism) {
  auto& s = mechanism.state();
  const auto& c = mechanism.config();

  const bool j1_allowed =
      s.joint1_at_alt || !c.joint12_interlock || !s.joint2_alt_reached;
  run_joint_stall(mechanism, j1_allowed, mechanism.motor1_a(), &mechanism.motor1_b(),
                  s.joint1_at_alt, c.joint1_alt_sign, s.joint1_alt_reached,
                  s.joint1_moving, s.joint1_stalled, s.joint1_timeout,
                  s.joint1_stall_since_ms, s.joint1_move_start_ms);

  const bool j2_allowed =
      !s.joint2_at_alt || !c.joint12_interlock || s.joint1_alt_reached;
  run_joint_stall(mechanism, j2_allowed, mechanism.motor2(), nullptr,
                  s.joint2_at_alt, c.joint2_alt_sign, s.joint2_alt_reached,
                  s.joint2_moving, s.joint2_stalled, s.joint2_timeout,
                  s.joint2_stall_since_ms, s.joint2_move_start_ms);
}

void apply_closed_loop(Arm2Dof& mechanism) {
  const auto& c = mechanism.config();
  // 闭环位置：固件位置环（非阻塞，每周期重新下发目标角）
  mechanism.motor1_a().spinToPosition(
      mechanism.state().joint1_target, c.position_units,
      c.position_speed_pct, vex::velocityUnits::pct, false);
  mechanism.motor1_b().spinToPosition(
      mechanism.state().joint1_target, c.position_units,
      c.position_speed_pct, vex::velocityUnits::pct, false);
  mechanism.motor2().spinToPosition(
      mechanism.state().joint2_target, c.position_units,
      c.position_speed_pct, vex::velocityUnits::pct, false);
}

}  // namespace

Arm2Dof::Arm2Dof(const Arm2DofConfig& config)
    : config_(config),
      motor1_a_(make_motor(config.motor1_a)),
      motor1_b_(make_motor(config.motor1_b)),
      motor2_(make_motor(config.motor2)) {
  state_.mode = config.mode;
  // 上电初始位置：目标 = home（编码器上电为 0）
  state_.joint1_target = config.joint1_home;
  state_.joint2_target = config.joint2_home;
  state_.joint1_at_alt = false;
  state_.joint2_at_alt = false;
  // 堵转找限位：假定上电时两关节都靠在 home 限位（收回位）
  state_.joint1_alt_reached = false;
  state_.joint2_alt_reached = false;
  // 单键整臂序列初始为空闲、整臂视为已收回
  state_.arm_extended = false;
  state_.arm_sequence_step = 0;
  // 停止模式持久化：任何来源的停止（含库内默认 coast）都按配置保持
  motor1_a_.setStopping(config.stop_brake_type);
  motor1_b_.setStopping(config.stop_brake_type);
  motor2_.setStopping(config.stop_brake_type);
}

vex::motor& Arm2Dof::motor1_a() { return motor1_a_; }
vex::motor& Arm2Dof::motor1_b() { return motor1_b_; }
vex::motor& Arm2Dof::motor2() { return motor2_; }
const vex::motor& Arm2Dof::motor1_a() const { return motor1_a_; }
const vex::motor& Arm2Dof::motor1_b() const { return motor1_b_; }
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
  // 仅单键整臂控制：Y 按下沿 → 收缩 ↔ 展开（自动按序）
  // （不占用单关节按键；单关节/开环指令可由 autonomous 自行构造 Command）
  command.arm_toggle = input.press_y;
  return command;
}

void arm_2dof_update(Arm2Dof& mechanism, const Arm2DofCommand& command) {
  if (mechanism.config().mode == Arm2DofMode::kClosedLoopPosition) {
    apply_position_edges(mechanism, command);  // 边沿 → 切换预设目标
    apply_closed_loop(mechanism);              // 位置环走向目标
  } else if (mechanism.config().mode == Arm2DofMode::kStallLimit) {
    apply_position_edges(mechanism, command);  // 手动单关节边沿 → 期望限位
    apply_arm_sequence(mechanism, command);    // 单键整臂 收缩↔展开 序列
    apply_stall_limit(mechanism);              // 开环定速 + 堵转判定到位
  } else {
    apply_open_loop(mechanism, command);
  }
  refresh_state(mechanism);
}

void arm_2dof_set_mode(Arm2Dof& mechanism, Arm2DofMode mode) {
  mechanism.config().mode = mode;
  mechanism.state().mode = mode;
}

void arm_2dof_set_extended(Arm2Dof& mechanism, bool extended) {
  auto& s = mechanism.state();
  if (extended) {
    s.arm_sequence_step = 1;  // 启动展开序列
    s.joint1_at_alt = true;
    s.joint1_timeout = false;
    s.joint2_at_alt = false;
  } else {
    s.arm_sequence_step = 2;  // 启动收回序列
    s.joint2_at_alt = false;
    s.joint2_timeout = false;
    s.joint1_at_alt = true;
  }
}

void arm_2dof_set_extend_allowed(Arm2Dof& mechanism, bool allowed) {
  mechanism.state().extend_allowed = allowed;
}

void arm_2dof_set_joint1_target(Arm2Dof& mechanism, double target) {
  mechanism.state().joint1_target = target;
}

void arm_2dof_set_joint2_target(Arm2Dof& mechanism, double target) {
  mechanism.state().joint2_target = target;
}

void arm_2dof_stop(Arm2Dof& mechanism, vex::brakeType brake_type) {
  stopcontrol(mechanism.motor1_a(), brake_type);
  stopcontrol(mechanism.motor1_b(), brake_type);
  stopcontrol(mechanism.motor2(), brake_type);
}

Arm2DofState& arm_2dof_state(Arm2Dof& mechanism) {
  return mechanism.state();
}

const Arm2DofState& arm_2dof_state(const Arm2Dof& mechanism) {
  return mechanism.state();
}

}  // namespace basic::mechanism
