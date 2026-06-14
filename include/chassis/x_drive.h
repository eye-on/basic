#ifndef BASIC_INCLUDE_X_DRIVE_H_
#define BASIC_INCLUDE_X_DRIVE_H_

#include "chassis/arcade_drive.h"

namespace basic::chassis {

/// 十字型全向轮底盘（X-Drive）控制指令
/// 支持前后、平移、旋转三自由度运动
struct XDriveCommand {
  int forward_input_pct{0};
  int strafe_input_pct{0};
  int turn_input_pct{0};
  vex::brakeType stop_brake_type{vex::coast};
};

/// 十字型全向轮底盘（X-Drive）运行状态
struct XDriveState {
  double fl_pct{0.0};
  double fr_pct{0.0};
  double bl_pct{0.0};
  double br_pct{0.0};
  vex::brakeType stop_brake_type{vex::coast};
};

/// 十字型全向轮底盘（X-Drive）配置
/// 四个角各有一个电机组，使用全向轮呈十字型（X 型）布局
template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
struct XDriveConfig {
  std::array<basic::device::MotorConfig, FlCount> fl_motors;  // 左前角电机组
  std::array<basic::device::MotorConfig, FrCount> fr_motors;  // 右前角电机组
  std::array<basic::device::MotorConfig, BlCount> bl_motors;  // 左后角电机组
  std::array<basic::device::MotorConfig, BrCount> br_motors;  // 右后角电机组
  int deadzone{10};
};

/// 十字型全向轮底盘（X-Drive）
/// 四个角各安装一组全向轮，呈 X 型布局，通过 mecanum 运动学实现全向移动
template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
class XDrive {
 public:
  explicit XDrive(
      const XDriveConfig<FlCount, FrCount, BlCount, BrCount>& config)
      : fl_motors_(detail::make_motor_array(config.fl_motors)),
        fr_motors_(detail::make_motor_array(config.fr_motors)),
        bl_motors_(detail::make_motor_array(config.bl_motors)),
        br_motors_(detail::make_motor_array(config.br_motors)),
        deadzone_(config.deadzone) {}

  /// 左前角（Front-Left）电机组访问
  std::array<vex::motor, FlCount>& fl_motors() {
    return fl_motors_;
  }

  const std::array<vex::motor, FlCount>& fl_motors() const {
    return fl_motors_;
  }

  /// 右前角（Front-Right）电机组访问
  std::array<vex::motor, FrCount>& fr_motors() {
    return fr_motors_;
  }

  const std::array<vex::motor, FrCount>& fr_motors() const {
    return fr_motors_;
  }

  /// 左后角（Back-Left）电机组访问
  std::array<vex::motor, BlCount>& bl_motors() {
    return bl_motors_;
  }

  const std::array<vex::motor, BlCount>& bl_motors() const {
    return bl_motors_;
  }

  /// 右后角（Back-Right）电机组访问
  std::array<vex::motor, BrCount>& br_motors() {
    return br_motors_;
  }

  const std::array<vex::motor, BrCount>& br_motors() const {
    return br_motors_;
  }

  int deadzone() const {
    return deadzone_;
  }

  XDriveState& state() {
    return state_;
  }

  const XDriveState& state() const {
    return state_;
  }

 private:
  std::array<vex::motor, FlCount> fl_motors_;
  std::array<vex::motor, FrCount> fr_motors_;
  std::array<vex::motor, BlCount> bl_motors_;
  std::array<vex::motor, BrCount> br_motors_;
  int deadzone_{10};
  XDriveState state_;
};

/// 初始化十字型全向轮底盘
template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
XDrive<FlCount, FrCount, BlCount, BrCount> x_drive_init(
    const XDriveConfig<FlCount, FrCount, BlCount, BrCount>& config) {
  return XDrive<FlCount, FrCount, BlCount, BrCount>(config);
}

/// 从遥控器输入生成十字型全向轮底盘控制指令
inline XDriveCommand x_drive_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input,
    ControllerAxis forward_axis,
    ControllerAxis strafe_axis,
    ControllerAxis turn_axis,
    vex::brakeType stop_brake_type = vex::coast) {
  return {
      controller_axis_value(input, forward_axis),
      controller_axis_value(input, strafe_axis),
      controller_axis_value(input, turn_axis),
      stop_brake_type,
  };
}

/// 设置十字型全向轮底盘各角电机输出
template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
void x_drive_set_output(
    XDrive<FlCount, FrCount, BlCount, BrCount>& chassis,
    double fl_pct,
    double fr_pct,
    double bl_pct,
    double br_pct,
    vex::brakeType brake_type) {
  chassis.state().fl_pct = fl_pct;
  chassis.state().fr_pct = fr_pct;
  chassis.state().bl_pct = bl_pct;
  chassis.state().br_pct = br_pct;
  chassis.state().stop_brake_type = brake_type;

  detail::apply_group_output(chassis.fl_motors(), fl_pct, brake_type);
  detail::apply_group_output(chassis.fr_motors(), fr_pct, brake_type);
  detail::apply_group_output(chassis.bl_motors(), bl_pct, brake_type);
  detail::apply_group_output(chassis.br_motors(), br_pct, brake_type);
}

/// 更新十字型全向轮底盘控制
/// 使用 mecanum 运动学将前后/平移/旋转指令分解为四个角的电机输出
template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
void x_drive_update(
    XDrive<FlCount, FrCount, BlCount, BrCount>& chassis,
    const XDriveCommand& command) {
  // 死区处理：输入在死区范围内则归零
  const int deadzone = chassis.deadzone();
  auto apply_deadzone = [deadzone](int input) -> double {
    return std::abs(input) > deadzone ? static_cast<double>(input) : 0.0;
  };

  const double forward = apply_deadzone(command.forward_input_pct);
  const double strafe = apply_deadzone(command.strafe_input_pct);
  const double turn = apply_deadzone(command.turn_input_pct);

  // X-drive mecanum 运动学分解：
  // fl = forward + strafe + turn
  // fr = forward - strafe - turn
  // bl = forward - strafe + turn
  // br = forward + strafe - turn
  double fl_pct = forward + strafe + turn;
  double fr_pct = forward - strafe - turn;
  double bl_pct = forward - strafe + turn;
  double br_pct = forward + strafe - turn;

  const double max_pct = std::max(
      {std::fabs(fl_pct), std::fabs(fr_pct),
       std::fabs(bl_pct), std::fabs(br_pct)});
  if (max_pct > 100.0) {
    const double scale = 100.0 / max_pct;
    fl_pct *= scale;
    fr_pct *= scale;
    bl_pct *= scale;
    br_pct *= scale;
  }

  x_drive_set_output(
      chassis,
      detail::shape_input(fl_pct),
      detail::shape_input(fr_pct),
      detail::shape_input(bl_pct),
      detail::shape_input(br_pct),
      command.stop_brake_type);
}

/// 停止十字型全向轮底盘
template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
void x_drive_stop(
    XDrive<FlCount, FrCount, BlCount, BrCount>& chassis,
    vex::brakeType brake_type = vex::coast) {
  x_drive_set_output(chassis, 0.0, 0.0, 0.0, 0.0, brake_type);
}

/// 获取十字型全向轮底盘状态（可修改）
template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
XDriveState& x_drive_state(
    XDrive<FlCount, FrCount, BlCount, BrCount>& chassis) {
  return chassis.state();
}

/// 获取十字型全向轮底盘状态（只读）
template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
const XDriveState& x_drive_state(
    const XDrive<FlCount, FrCount, BlCount, BrCount>& chassis) {
  return chassis.state();
}

}  // namespace basic::chassis

#endif  // BASIC_INCLUDE_X_DRIVE_H_
