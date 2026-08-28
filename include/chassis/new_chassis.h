#ifndef BASIC_INCLUDE_NEW_CHASSIS_H_
#define BASIC_INCLUDE_NEW_CHASSIS_H_

#include "chassis/steering/steering_drive.h"

namespace basic::chassis {

using NewChassis = basic::chassis::steering::SteeringDrive;
using NewChassisCommand = basic::chassis::steering::ArcadeDriveCommand;
using NewChassisConfig = basic::chassis::steering::SteeringDriveConfig;
using NewChassisState = basic::chassis::steering::SteeringDriveState;
using WheelUnit = basic::chassis::steering::WheelUnit;
using WheelUnitConfig = basic::chassis::steering::WheelUnitConfig;

inline NewChassis new_chassis_init(const NewChassisConfig& config) {
  return basic::chassis::steering::steering_init(config);
}

inline NewChassisCommand new_chassis_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input,
    vex::brakeType stop_brake_type = vex::coast) {
  // VEX 手柄轴编号：axis1=右摇杆左右, axis2=右摇杆上下,
  //                 axis3=左摇杆上下, axis4=左摇杆左右
  // 映射（与 PROS 版一致）：轴3（左Y）= 前后 vx，轴4（左X）= 平移 vy，轴1（右X）= 旋转 omega
  // 量程对齐：PROS get_analog 原始值域 ±127，basic position(pct) 为 ±100，
  // 乘 127/100 使两边同杆位油门手感一致（满杆 127% 会顶到电机饱和，与 PROS 相同）
  constexpr double kAxisScaleToPros = 127.0 / 100.0;
  const double vx_pct = static_cast<double>(
                            basic::chassis::steering::controller_axis_value(
                                input,
                                basic::chassis::steering::ControllerAxis::kAxis3)) *
                        kAxisScaleToPros;
  const double vy_pct = static_cast<double>(
                            basic::chassis::steering::controller_axis_value(
                                input,
                                basic::chassis::steering::ControllerAxis::kAxis4)) *
                        kAxisScaleToPros;
  const double omega_pct = static_cast<double>(
                               basic::chassis::steering::controller_axis_value(
                                   input,
                                   basic::chassis::steering::ControllerAxis::kAxis1)) *
                           kAxisScaleToPros;

  return {vx_pct, vy_pct, omega_pct, stop_brake_type};
}

inline void new_chassis_update(
    NewChassis& chassis,
    const NewChassisCommand& command) {
  basic::chassis::steering::steering_update(chassis, command);
}

inline void new_chassis_stop(
    NewChassis& chassis,
    vex::brakeType brake_type = vex::coast) {
  // 力控架构：停止 = 全部电机零电压（自然滑行），与 PROS 版一致
  (void)brake_type;
  chassis.stop();
}

/// 机械标零：将底盘四个舵轮当前物理位置记录为航向 0°
inline void new_chassis_calibrate_zero(NewChassis& chassis) {
  basic::chassis::steering::steering_calibrate_zero(chassis);
}

/// 物理回正：四轮回到各自标零位置（上电后调用一次；未完成需周期调用）
inline bool new_chassis_align_to_physical_zero(NewChassis& chassis) {
  return chassis.align_to_physical_zero();
}

inline NewChassisState& new_chassis_state(NewChassis& chassis) {
  return chassis.state();
}

inline const NewChassisState& new_chassis_state(const NewChassis& chassis) {
  return chassis.state();
}

inline WheelUnit& new_chassis_fr(NewChassis& chassis) {
  return chassis.fr();
}

inline const WheelUnit& new_chassis_fr(const NewChassis& chassis) {
  return chassis.fr();
}

inline WheelUnit& new_chassis_fl(NewChassis& chassis) {
  return chassis.fl();
}

inline const WheelUnit& new_chassis_fl(const NewChassis& chassis) {
  return chassis.fl();
}

inline WheelUnit& new_chassis_br(NewChassis& chassis) {
  return chassis.br();
}

inline const WheelUnit& new_chassis_br(const NewChassis& chassis) {
  return chassis.br();
}

inline WheelUnit& new_chassis_bl(NewChassis& chassis) {
  return chassis.bl();
}

inline const WheelUnit& new_chassis_bl(const NewChassis& chassis) {
  return chassis.bl();
}

}  // namespace basic::chassis

#endif  // BASIC_INCLUDE_NEW_CHASSIS_H_
