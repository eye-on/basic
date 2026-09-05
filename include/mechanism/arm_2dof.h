#ifndef BASIC_INCLUDE_MECHANISM_ARM_2DOF_H_
#define BASIC_INCLUDE_MECHANISM_ARM_2DOF_H_

#include "device_config.h"
#include "hardware/shared/state_types.h"

namespace basic::mechanism {

/// 二自由度机械臂控制模式
enum class Arm2DofMode {
  kOpenLoopVelocity,   // 开环速度：按键按住 = 以配置速度持续转动（暂时使用）
  kClosedLoopPosition, // 闭环位置：由 arm_2dof_set_joint*_target 给定目标角，
                       // 走电机固件位置环（spinToPosition）
};

struct Arm2DofConfig {
  basic::device::MotorConfig motor1;       // 关节 1 电机
  basic::device::MotorConfig motor2;       // 关节 2 电机
  Arm2DofMode mode{Arm2DofMode::kOpenLoopVelocity};  // TODO: 位置闭环就绪后切回/配置
  double velocity_speed_pct{40.0};         // 开环速度幅值（按住按键时）
  vex::rotationUnits position_units{vex::deg};  // 闭环位置单位
  double position_speed_pct{30.0};         // 闭环运动速度
};

struct Arm2DofCommand {
  // 开环指令：±1 = 方向，0 = 停（由 command_from_controller 从按键电平生成）
  int joint1_direction{0};  // up=+1 / down=-1
  int joint2_direction{0};  // x=+1 / b=-1
};

struct Arm2DofState {
  Arm2DofMode mode{Arm2DofMode::kOpenLoopVelocity};
  double joint1_position{0.0};  // 关节 1 电机当前位置（position_units）
  double joint2_position{0.0};  // 关节 2 电机当前位置
  double joint1_target{0.0};    // 闭环目标（position_units）
  double joint2_target{0.0};
  int joint1_direction{0};      // 最近开环指令方向
  int joint2_direction{0};
};

class Arm2Dof {
 public:
  explicit Arm2Dof(const Arm2DofConfig& config);

  vex::motor& motor1();
  vex::motor& motor2();
  const vex::motor& motor1() const;
  const vex::motor& motor2() const;

  Arm2DofConfig& config();
  const Arm2DofConfig& config() const;

  Arm2DofState& state();
  const Arm2DofState& state() const;

 private:
  Arm2DofConfig config_;
  vex::motor motor1_;
  vex::motor motor2_;
  Arm2DofState state_;
};

/// 初始化二自由度机械臂
Arm2Dof arm_2dof_init(const Arm2DofConfig& config);

/// 从遥控器输入生成指令（开环键位）：
/// 上/下（方向键）= 关节1（+/-），X/B = 关节2（+/-）
Arm2DofCommand arm_2dof_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input);

/// 主更新：按模式驱动（开环 = 电平方向速度；闭环 = 固件位置环到目标角）
void arm_2dof_update(Arm2Dof& mechanism, const Arm2DofCommand& command);

/// 切换控制模式（闭环模式需先设目标角）
void arm_2dof_set_mode(Arm2Dof& mechanism, Arm2DofMode mode);

/// 设置闭环目标角（position_units；autonomous 用）
void arm_2dof_set_joint1_target(Arm2Dof& mechanism, double target);
void arm_2dof_set_joint2_target(Arm2Dof& mechanism, double target);

/// 停止机械臂（默认 hold：手动模式松键后保持姿态）
void arm_2dof_stop(Arm2Dof& mechanism, vex::brakeType brake_type = vex::hold);

/// 状态访问（mutable / const）
Arm2DofState& arm_2dof_state(Arm2Dof& mechanism);
const Arm2DofState& arm_2dof_state(const Arm2Dof& mechanism);

}  // namespace basic::mechanism

#endif  // BASIC_INCLUDE_MECHANISM_ARM_2DOF_H_
