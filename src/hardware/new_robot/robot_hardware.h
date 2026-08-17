#ifndef BASIC_SRC_HARDWARE_NEW_ROBOT_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_NEW_ROBOT_ROBOT_HARDWARE_H_

#include "chassis/new_chassis.h"
#include "vex.h"

namespace basic::hardware::new_robot {

inline constexpr int kRefreshTime = 10;
inline constexpr int kSensorLoopDelay = 50;

// 三线模拟编码器（模拟输入 0-4095，绝对位置，断电保持航向）
// 每个占 1 个三线口（A-H）；分配：FR→A, FL→B, BR→C, BL→D（按实际接线修改）
inline constexpr double kAnalogFullScaleDeg = 360.0;  // 0-4095 满量程对应角度行程
inline constexpr bool kAnalogReversed = false;        // 装车后若航向读数反向，改为 true
inline constexpr double kAnalogZeroRaw = 0.0;         // 三线编码基准（raw），0 = 上电自动捕获当前读数
inline constexpr double kAnalogDeadbandRaw = 5.0;     // 模拟读数滞环死区（raw，±1 = ±0.088°）
inline constexpr double kMotorMaxRpm = 600.0;        // 轮组电机极速（ratio6_1 蓝盒 = 600rpm）

inline constexpr basic::control::pid::Pid::Config kVeloPidCfg{
    10.0, 0.0, 0.0,
    -200.0, 200.0, -200.0, 200.0, 1};   

//{30, 0.5, 0.0,-200.0, 200.0, -50.0, 50.0, 0.05}

inline constexpr basic::control::pid::Pid::Config kHeadingPidCfg{
    1.0, 0.0, 0.001,
    -25.0, 25.0, -1000.0, 1000.0, 1};  

inline constexpr basic::control::pid::Pid::Config kAngularVelocityPidCfg{
    2.0, 0.05, 0.0,
    -200.0, 200.0, -10.0, 10.0, 1};   
inline constexpr first_order_adrc::Controller::Config kAdrcCfg;

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{vex::PORT11};
  basic::chassis::NewChassis new_chassis;

  RobotHardware() //fr fl br bl
      : new_chassis(basic::chassis::new_chassis_init({
            {
                {vex::PORT3, vex::ratio6_1, true},
                {vex::PORT2, vex::ratio6_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
                kMotorMaxRpm,     // motor_max_rpm（ratio6_1 蓝盒）
                0.0,             // initial_angle_a（0 = 自动捕获）
                0.0,             // initial_angle_b（0 = 自动捕获）
                &brain.ThreeWirePort.A,  // analog_port（三线模拟编码器）
                kAnalogFullScaleDeg,
                kAnalogReversed,
                kAnalogZeroRaw,
                kAnalogDeadbandRaw,
                0,  // debug_id = FR
            },
            {
                {vex::PORT12, vex::ratio6_1, true},
                {vex::PORT13, vex::ratio6_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
                kMotorMaxRpm,
                0.0,
                0.0,
                nullptr,  // analog_port（未接编码器：跳过对齐/航向反馈）
                kAnalogFullScaleDeg,
                kAnalogReversed,
                kAnalogZeroRaw,
                kAnalogDeadbandRaw,
                1,  // debug_id = FL
            },
            {
                {vex::PORT10, vex::ratio6_1, true},
                {vex::PORT9, vex::ratio6_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
                kMotorMaxRpm,
                0.0,
                0.0,
                nullptr,  // analog_port（未接编码器）
                kAnalogFullScaleDeg,
                kAnalogReversed,
                kAnalogZeroRaw,
                kAnalogDeadbandRaw,
                2,  // debug_id = BR
            },
            {
                {vex::PORT19, vex::ratio6_1, true},
                {vex::PORT18, vex::ratio6_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
                kMotorMaxRpm,
                0.0,
                0.0,
                nullptr,  // analog_port（未接编码器）
                kAnalogFullScaleDeg,
                kAnalogReversed,
                kAnalogZeroRaw,
                kAnalogDeadbandRaw,
                3,  // debug_id = BL
            },
            2,
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
