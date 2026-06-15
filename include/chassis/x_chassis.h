#ifndef BASIC_INCLUDE_X_CHASSIS_H_
#define BASIC_INCLUDE_X_CHASSIS_H_

#include "chassis/x_drive.h"

namespace basic::chassis {

/// 十字型全向轮底盘类型别名：四个角各 1 个电机
using XChassis = XDrive<1, 1, 1, 1>;
using XChassisCommand = XDriveCommand;
using XChassisConfig = XDriveConfig<1, 1, 1, 1>;
using XChassisState = XDriveState;
using XChassisOdometry = XDriveOdometry;

/// 初始化十字型全向轮底盘
inline XChassis x_chassis_init(const XChassisConfig& config) {
  return x_drive_init(config);
}

/// 从遥控器输入生成十字型全向轮底盘控制指令
/// 默认映射：左摇杆 Y（轴2）= 前后，左摇杆 X（轴1）= 平移，右摇杆 X（轴4）= 旋转
inline XChassisCommand x_chassis_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input,
    vex::brakeType stop_brake_type = vex::coast) {
  return x_drive_command_from_controller(
      input,
      ControllerAxis::kAxis2,   // 左摇杆 Y → 前后
      ControllerAxis::kAxis1,   // 左摇杆 X → 平移
      ControllerAxis::kAxis4,   // 右摇杆 X → 旋转
      stop_brake_type);
}

/// 更新十字型全向轮底盘
inline void x_chassis_update(
    XChassis& chassis,
    const XChassisCommand& command) {
  x_drive_update(chassis, command);
}

/// 停止十字型全向轮底盘
inline void x_chassis_stop(
    XChassis& chassis,
    vex::brakeType brake_type = vex::coast) {
  x_drive_stop(chassis, brake_type);
}

/// 获取十字型全向轮底盘状态（可修改）
inline XChassisState& x_chassis_state(XChassis& chassis) {
  return x_drive_state(chassis);
}

/// 获取十字型全向轮底盘状态（只读）
inline const XChassisState& x_chassis_state(const XChassis& chassis) {
  return x_drive_state(chassis);
}

// ──────────────────────────────────────────────
// 各角电机访问器（完整名称）
// ──────────────────────────────────────────────

/// 左前（Front-Left）电机
inline XChassisOdometry& x_chassis_odometry(XChassis& chassis) {
  return x_drive_odometry(chassis);
}

inline const XChassisOdometry& x_chassis_odometry(const XChassis& chassis) {
  return x_drive_odometry(chassis);
}

inline void x_chassis_reset_odometry(
    XChassis& chassis,
    double x_m = 0.0,
    double y_m = 0.0) {
  x_drive_reset_odometry(chassis, x_m, y_m);
}

inline double x_chassis_x_m(const XChassis& chassis) {
  return x_chassis_odometry(chassis).x_m;
}

inline double x_chassis_y_m(const XChassis& chassis) {
  return x_chassis_odometry(chassis).y_m;
}

inline vex::motor& x_chassis_fl_motor(XChassis& chassis) {
  return chassis.fl_motors()[0];
}

inline const vex::motor& x_chassis_fl_motor(const XChassis& chassis) {
  return chassis.fl_motors()[0];
}

/// 右前（Front-Right）电机
inline vex::motor& x_chassis_fr_motor(XChassis& chassis) {
  return chassis.fr_motors()[0];
}

inline const vex::motor& x_chassis_fr_motor(const XChassis& chassis) {
  return chassis.fr_motors()[0];
}

/// 左后（Back-Left）电机
inline vex::motor& x_chassis_bl_motor(XChassis& chassis) {
  return chassis.bl_motors()[0];
}

inline const vex::motor& x_chassis_bl_motor(const XChassis& chassis) {
  return chassis.bl_motors()[0];
}

/// 右后（Back-Right）电机
inline vex::motor& x_chassis_br_motor(XChassis& chassis) {
  return chassis.br_motors()[0];
}

inline const vex::motor& x_chassis_br_motor(const XChassis& chassis) {
  return chassis.br_motors()[0];
}

// ──────────────────────────────────────────────
// 各角电机访问器（简短别名）
// ──────────────────────────────────────────────

/// 左前（Front-Left）电机简短别名
inline vex::motor& x_chassis_fl(XChassis& chassis) {
  return x_chassis_fl_motor(chassis);
}

inline const vex::motor& x_chassis_fl(const XChassis& chassis) {
  return x_chassis_fl_motor(chassis);
}

/// 右前（Front-Right）电机简短别名
inline vex::motor& x_chassis_fr(XChassis& chassis) {
  return x_chassis_fr_motor(chassis);
}

inline const vex::motor& x_chassis_fr(const XChassis& chassis) {
  return x_chassis_fr_motor(chassis);
}

/// 左后（Back-Left）电机简短别名
inline vex::motor& x_chassis_bl(XChassis& chassis) {
  return x_chassis_bl_motor(chassis);
}

inline const vex::motor& x_chassis_bl(const XChassis& chassis) {
  return x_chassis_bl_motor(chassis);
}

/// 右后（Back-Right）电机简短别名
inline vex::motor& x_chassis_br(XChassis& chassis) {
  return x_chassis_br_motor(chassis);
}

inline const vex::motor& x_chassis_br(const XChassis& chassis) {
  return x_chassis_br_motor(chassis);
}

}  // namespace basic::chassis

#endif  // BASIC_INCLUDE_X_CHASSIS_H_
