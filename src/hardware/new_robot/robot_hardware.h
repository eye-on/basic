#ifndef BASIC_SRC_HARDWARE_NEW_ROBOT_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_NEW_ROBOT_ROBOT_HARDWARE_H_

#include "chassis/new_chassis.h"
#include "vex.h"

namespace basic::hardware::new_robot {

inline constexpr int kRefreshTime = 10;
inline constexpr int kSensorLoopDelay = 50;

// 三线模拟编码器（模拟输入 0-4095，绝对位置，断电保持航向）
// 每个占 1 个三线口（A-H）；分配（实际接线）：FR→A, FL→C, BR→B, BL→D
inline constexpr double kAnalogFullScaleDeg = 360.0;  // 0-4095 满量程对应角度行程
inline constexpr bool kAnalogReversed = false;        // 装车后若航向读数反向，改为 true
// 标零位置已持久化（最新实测 zero 值）：上电物理回正将转到这些位置
inline constexpr double kAnalogZeroRawFr = 3663.0;    // 右前，ADI A
inline constexpr double kAnalogZeroRawFl = 1310.0;    // 左前，ADI C
inline constexpr double kAnalogZeroRawBr = 3535.0;    // 右后，ADI B
inline constexpr double kAnalogZeroRawBl = 2416.0;    // 左后，ADI D
inline constexpr double kAnalogDeadbandRaw = 8.0;     // 模拟读数滞环死区（raw，±8 = ±0.70°）
inline constexpr double kMotorMaxRpm = 600.0;        // 轮组电机极速（ratio6_1 蓝盒 = 600rpm）
inline constexpr double kAlignToleranceDeg = 1.0;     // 物理回正容差（度）
inline constexpr int kAlignTimeoutMs = 2500;          // 物理回正超时（毫秒）

// 力控（电压直驱）：kp=10, ki=0.3（消除负载稳态误差），输出 ±200，积分 ±50，死区 0.1 m/s
inline constexpr basic::control::pid::Pid::Config kVeloPidCfg{
    10.0, 0.3, 0.0,
    -200.0, 200.0, -50.0, 50.0, 0.1};

// 航向位置环：kp=1.0，输出 ±25（°/s 目标），死区 0.5°
inline constexpr basic::control::pid::Pid::Config kHeadingPidCfg{
    1.0, 0.0, 0.0,
    -25.0, 25.0, -1000.0, 1000.0, 0.5};

// 转向角速度环：kp=4.0，输出 ±200，死区 0.5
inline constexpr basic::control::pid::Pid::Config kAngularVelocityPidCfg{
    4.0, 0.0, 0.0,
    -200.0, 200.0, -10.0, 10.0, 0.5};
inline constexpr first_order_adrc::Controller::Config kAdrcCfg;

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{vex::PORT11};
  basic::chassis::NewChassis new_chassis;

  RobotHardware() //fr fl br bl（实际接线：FR=7/8, FL=5/6, BR=1/13, BL=3/4）
      : new_chassis(basic::chassis::new_chassis_init({
            {
                {vex::PORT7, vex::ratio6_1, true},
                {vex::PORT8, vex::ratio6_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
                kMotorMaxRpm,     // motor_max_rpm（ratio6_1 蓝盒）
                0.0,             // initial_angle_a（0 = 自动捕获）
                0.0,             // initial_angle_b（0 = 自动捕获）
                &brain.ThreeWirePort.A,  // analog_port（FR→A）
                kAnalogFullScaleDeg,
                kAnalogReversed,
                kAnalogZeroRawFr,  // 持久标零：3663
                kAnalogDeadbandRaw,
                0,  // debug_id = FR
                kAlignToleranceDeg,
                kAlignTimeoutMs,
            },
            {
                {vex::PORT5, vex::ratio6_1, true},
                {vex::PORT6, vex::ratio6_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
                kMotorMaxRpm,
                0.0,
                0.0,
                &brain.ThreeWirePort.C,  // analog_port（FL→C）
                kAnalogFullScaleDeg,
                kAnalogReversed,
                kAnalogZeroRawFl,  // 持久标零：1310
                kAnalogDeadbandRaw,
                1,  // debug_id = FL
                kAlignToleranceDeg,
                kAlignTimeoutMs,
            },
            {
                {vex::PORT1, vex::ratio6_1, true},
                {vex::PORT13, vex::ratio6_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
                kMotorMaxRpm,
                0.0,
                0.0,
                &brain.ThreeWirePort.B,  // analog_port（BR→B）
                kAnalogFullScaleDeg,
                kAnalogReversed,
                kAnalogZeroRawBr,  // 持久标零：3535
                kAnalogDeadbandRaw,
                2,  // debug_id = BR
                kAlignToleranceDeg,
                kAlignTimeoutMs,
            },
            {
                {vex::PORT3, vex::ratio6_1, true},
                {vex::PORT4, vex::ratio6_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
                kMotorMaxRpm,
                0.0,
                0.0,
                &brain.ThreeWirePort.D,  // analog_port（BL→D）
                kAnalogFullScaleDeg,
                kAnalogReversed,
                kAnalogZeroRawBl,  // 持久标零：2416
                kAnalogDeadbandRaw,
                3,  // debug_id = BL
                kAlignToleranceDeg,
                kAlignTimeoutMs,
            },
            3,  // 摇杆死区（±3 pct）
        })) {}

  void calibrate_inertial_sensor() {
    inertial.calibrate();
    while (inertial.isCalibrating()) {
      vex::wait(5, vex::msec);
    }

    inertial.resetHeading();
    inertial.resetRotation();
  }

  void show_calibrated() {
    controller.Screen.setCursor(5, 1);
    controller.Screen.print("      calibrated!");
  }
};

}  // namespace basic::hardware::new_robot

#endif  // BASIC_SRC_HARDWARE_NEW_ROBOT_ROBOT_HARDWARE_H_
