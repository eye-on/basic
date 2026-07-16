#ifndef BASIC_SRC_HARDWARE_LOOKLOOK_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_LOOKLOOK_ROBOT_HARDWARE_H_

#include "chassis/x_chassis.h"
#include "mechanism/gripper.h"
#include "mechanism/linear_lift.h"
#include "vex.h"

namespace basic::hardware::looklook {

inline constexpr int kRefreshTime = 10;

// 底盘端口
inline const int kFrontLeftMotorPort = vex::PORT9;
inline const int kFrontRightMotorPort = vex::PORT17;
inline const int kBackLeftMotorPort = vex::PORT7;
inline const int kBackRightMotorPort = vex::PORT1;

// 抬升端口
inline const int kLiftMotor1Port = vex::PORT20;
inline const int kLiftMotor2Port = vex::PORT6;

// 夹爪端口
inline const int kGripperMotorPort = vex::PORT8;

// 传感器端口
inline const int kInertialPort = vex::PORT11;

// 底盘参数
inline constexpr int kDriveWheelTrackMm = 10;
inline constexpr basic::control::pid::Pid::Config kDefaultPid{};

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{kInertialPort};
  basic::chassis::XChassis x_chassis;
  basic::mechanism::LinearLift lift;
  basic::mechanism::Gripper gripper;

  RobotHardware()
      : x_chassis(basic::chassis::x_chassis_init({
            {{ {kFrontLeftMotorPort, vex::ratio6_1, false} }},   // 左前 FL
            {{ {kFrontRightMotorPort, vex::ratio6_1, true} }}, // 右前 FR
            {{ {kBackLeftMotorPort, vex::ratio6_1, false} }},    // 左后 BL
            {{ {kBackRightMotorPort, vex::ratio6_1, true} }},  // 右后 BR
            kDriveWheelTrackMm,
            {{kDefaultPid}}, {{kDefaultPid}}, {{kDefaultPid}}, {{kDefaultPid}},
        })),
        lift(basic::mechanism::linear_lift_init({
            {{kLiftMotor1Port, vex::ratio18_1, true}, 0, 500},
            {{kLiftMotor2Port, vex::ratio18_1, true}, 0, 350},
            60.0,   // closed_loop_speed_pct
            20.0,   // open_loop_speed_pct
            60.0,   // closed_loop_speed_down_pct
            15.0,   // open_loop_speed_down_pct
            vex::deg, 50.0, 50.0,  // position_units, sync, decel
            vex::hold,          // stop_brake_type
        })),
        gripper(basic::mechanism::gripper_init({
            {{kGripperMotorPort, vex::ratio6_1, false}},
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

}  // namespace basic::hardware::looklook

#endif  // BASIC_SRC_HARDWARE_LOOKLOOK_ROBOT_HARDWARE_H_
