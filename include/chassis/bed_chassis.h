#ifndef BASIC_INCLUDE_BED_CHASSIS_H_
#define BASIC_INCLUDE_BED_CHASSIS_H_

#include "chassis/x_drive.h"

namespace basic::chassis {

/// 十字型全向轮底盘（双电机版）类型别名：四个角各 2 电机驱动 1 个轮，共 8 电机
using BedChassis = XDrive<2, 2, 2, 2>;
using BedChassisCommand = XDriveCommand;
using BedChassisConfig = XDriveConfig<2, 2, 2, 2>;
using BedChassisState = XDriveState;
using BedChassisOdometry = XDriveOdometry;

/// 初始化双电机十字型全向轮底盘
inline BedChassis bed_chassis_init(const BedChassisConfig& config) {
  return x_drive_init(config);
}

/// 从遥控器输入生成底盘控制指令
/// 映射（与 x_chassis 一致）：左摇杆 Y（轴2）= 前后，左摇杆 X（轴1）= 平移，
/// 右摇杆 X（轴4）= 旋转
inline BedChassisCommand bed_chassis_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input,
    vex::brakeType stop_brake_type = vex::coast) {
  return x_drive_command_from_controller(
      input,
      ControllerAxis::kAxis2,
      ControllerAxis::kAxis1,
      ControllerAxis::kAxis4,
      stop_brake_type);
}

/// 更新双电机十字型全向轮底盘（同组 2 电机下发同一输出）
inline void bed_chassis_update(
    BedChassis& chassis,
    const BedChassisCommand& command) {
  x_drive_update(chassis, command);
}

/// 停止双电机十字型全向轮底盘
inline void bed_chassis_stop(
    BedChassis& chassis,
    vex::brakeType brake_type = vex::coast) {
  x_drive_stop(chassis, brake_type);
}

/// 获取底盘状态（可修改 / 只读）
inline BedChassisState& bed_chassis_state(BedChassis& chassis) {
  return x_drive_state(chassis);
}

inline const BedChassisState& bed_chassis_state(const BedChassis& chassis) {
  return x_drive_state(chassis);
}

// ──────────────────────────────────────────────
// 里程计
// ──────────────────────────────────────────────

inline BedChassisOdometry& bed_chassis_odometry(BedChassis& chassis) {
  return x_drive_odometry(chassis);
}

inline const BedChassisOdometry& bed_chassis_odometry(const BedChassis& chassis) {
  return x_drive_odometry(chassis);
}

inline void bed_chassis_reset_odometry(
    BedChassis& chassis,
    double x_m = 0.0,
    double y_m = 0.0) {
  x_drive_reset_odometry(chassis, x_m, y_m);
}

}  // namespace basic::chassis

#endif  // BASIC_INCLUDE_BED_CHASSIS_H_
