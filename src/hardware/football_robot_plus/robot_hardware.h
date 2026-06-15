#ifndef BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_ROBOT_HARDWARE_H_

#include "chassis/x_chassis.h"
#include "hardware/football_robot_plus/external_vision_serial.h"
#include "vex.h"

namespace basic::hardware::football_robot_plus {

inline constexpr int kRefreshTime = 10;
inline constexpr double kDriveOutputLimitPct = 50.0;
inline const int kFrontLeftMotorPort = vex::PORT1;
inline const int kBackLeftMotorPort = vex::PORT12;
inline const int kFrontRightMotorPort = vex::PORT19;
inline const int kBackRightMotorPort = vex::PORT10;
/// 左前轮（Front-Left）PID 参数
inline constexpr basic::control::pid::Pid::Config kFlPidCfg{
    0.5, 0.0, 0.0,
    -kDriveOutputLimitPct, kDriveOutputLimitPct, -10.0, 10.0, 1,
    basic::control::pid::Type::kPosition};

/// 右前轮（Front-Right）PID 参数
inline constexpr basic::control::pid::Pid::Config kFrPidCfg{
    0.5, 0.0, 0.0,
    -kDriveOutputLimitPct, kDriveOutputLimitPct, -10.0, 10.0, 1,
    basic::control::pid::Type::kPosition};
    
/// 左后轮（Back-Left）PID 参数
inline constexpr basic::control::pid::Pid::Config kBlPidCfg{
    0.5, 0.0, 0.0,
    -kDriveOutputLimitPct, kDriveOutputLimitPct, -10.0, 10.0, 1,
    basic::control::pid::Type::kPosition};

/// 右后轮（Back-Right）PID 参数
inline constexpr basic::control::pid::Pid::Config kBrPidCfg{
    0.5, 0.0, 0.0,
    -kDriveOutputLimitPct, kDriveOutputLimitPct, -10.0, 10.0, 1,
    basic::control::pid::Type::kPosition};

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{vex::PORT11};
  ExternalVisionSerial external_vision;
  basic::chassis::XChassis football_chassis;

  RobotHardware()
      : external_vision(kExternalVisionPort),
        football_chassis(basic::chassis::x_chassis_init({
            {{
                {kFrontLeftMotorPort, vex::ratio18_1, true},
            }},
            {{
                {kFrontRightMotorPort, vex::ratio18_1, false},
            }},
            {{
                {kBackLeftMotorPort, vex::ratio18_1, true},
            }},
            {{
                {kBackRightMotorPort, vex::ratio18_1, false},
            }},
            2,
            {{kFlPidCfg}},
            {{kFrPidCfg}},
            {{kBlPidCfg}},
            {{kBrPidCfg}},
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
    controller.Screen.setCursor(3, 1);
    controller.Screen.print("CALIBRATED         ");
  }
};

}  // namespace basic::hardware::football_robot_plus

#endif  // BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_ROBOT_HARDWARE_H_
