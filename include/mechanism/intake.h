#ifndef BASIC_INCLUDE_MECHANISM_INTAKE_H_
#define BASIC_INCLUDE_MECHANISM_INTAKE_H_

#include "device_config.h"
#include "hardware/shared/state_types.h"

namespace basic::mechanism {

/// 吸入模块模式：停止 / 吸入（开环定速）
enum class IntakeMode {
  kOff,
  kRunning,
};

struct IntakeConfig {
  basic::device::MotorConfig motor_a;  // 驱动电机 A（两个电机同向驱动吸入轮）
  basic::device::MotorConfig motor_b;  // 驱动电机 B
  double speed_pct{100.0};             // 开环目标速度（pct，直接下发不经 PID）
};

struct IntakeCommand {
  bool toggle{false};  // press_l1 → 边沿触发翻转 开/停
};

struct IntakeState {
  IntakeMode mode{IntakeMode::kOff};
  double speed_pct{0.0};  // 当前下发速度（开环）
};

class Intake {
 public:
  explicit Intake(const IntakeConfig& config);

  vex::motor& motor_a();
  vex::motor& motor_b();
  const vex::motor& motor_a() const;
  const vex::motor& motor_b() const;

  IntakeConfig& config();
  const IntakeConfig& config() const;

  IntakeState& state();
  const IntakeState& state() const;

 private:
  IntakeConfig config_;
  vex::motor motor_a_;
  vex::motor motor_b_;
  IntakeState state_;
};

/// 初始化吸入模块
Intake intake_init(const IntakeConfig& config);

/// 从遥控器输入生成控制指令（键位：L1 = 边沿启停）
IntakeCommand intake_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input);

/// 主更新：处理 toggle 边沿，按模式开环下发速度
void intake_update(Intake& mechanism, const IntakeCommand& command);

/// 直接设置运行/停止（autonomous 用；速度取配置值）
void intake_set_running(Intake& mechanism, bool running);

/// 停止吸入模块并重置状态
void intake_stop(Intake& mechanism, vex::brakeType brake_type = vex::coast);

/// 状态访问（mutable / const）
IntakeState& intake_state(Intake& mechanism);
const IntakeState& intake_state(const Intake& mechanism);

}  // namespace basic::mechanism

#endif  // BASIC_INCLUDE_MECHANISM_INTAKE_H_
