#ifndef BASIC_INCLUDE_MECHANISM_GRIPPER_H_
#define BASIC_INCLUDE_MECHANISM_GRIPPER_H_

#include "device_config.h"
#include "hardware/shared/state_types.h"

namespace basic::mechanism {

/// 夹爪模式：松开 / 夹住
enum class GripperMode {
  kOpen,
  kClosed,
};

/// 夹爪电机槽位：含软件限位
struct GripperMotorSlot {
  basic::device::MotorConfig motor;
  double position_min{0.0};  // 松开位置（行程下限）
  double position_max{0.0};  // 夹住位置（行程上限）
};

struct GripperConfig {
  GripperMotorSlot motor;
  double speed_pct{60.0};                       // 闭环运动速度
  double clamp_torque_nm{0.3};                  // 夹紧保持力矩（Nm），正值朝行程上限（夹住）方向
  vex::rotationUnits position_units{vex::deg};  // 位置单位
};

struct GripperCommand {
  bool toggle{false};  // press_r2 → 边沿触发翻转开/关
};

struct GripperState {
  // 开局夹爪物理上处于夹紧位置：初始模式为夹住，初始化时编码器对齐到行程上限
  GripperMode mode{GripperMode::kClosed};
  double motor_position{0.0};
  bool at_target{true};
};

class Gripper {
 public:
  explicit Gripper(const GripperConfig& config);

  vex::motor& motor();
  const vex::motor& motor() const;

  GripperConfig& config();
  const GripperConfig& config() const;

  GripperState& state();
  const GripperState& state() const;

 private:
  GripperConfig config_;
  vex::motor motor_;
  GripperState state_;
};

/// 初始化夹爪
Gripper gripper_init(const GripperConfig& config);

/// 从遥控器输入生成夹爪控制指令
GripperCommand gripper_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input);

/// 主更新：处理 toggle 并驱动电机到目标位置
void gripper_update(Gripper& mechanism, const GripperCommand& command);

/// 停止夹爪并重置状态
void gripper_stop(Gripper& mechanism, vex::brakeType brake_type = vex::hold);

/// 状态访问（mutable / const）
GripperState& gripper_state(Gripper& mechanism);
const GripperState& gripper_state(const Gripper& mechanism);

}  // namespace basic::mechanism

#endif
