#ifndef BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_ROBOT_HARDWARE_H_

#include "chassis/new_chassis.h"
#include "mechanism/single_pneumatic.h"
#include "vex.h"

namespace basic::hardware::football_robot {

inline constexpr int kRefreshTime = 10;
inline constexpr double kDriveOutputLimitPct = 30.0;
inline const int kFrontLeftMotorPort = vex::PORT3;
inline const int kBackLeftMotorPort = vex::PORT7;
inline const int kFrontRightMotorPort = vex::PORT12;
inline const int kBackRightMotorPort = vex::PORT5;
inline const int kCenterStrafeMotorPort = vex::PORT20;

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{vex::PORT11};
  basic::chassis::NewChassis football_chassis;
  basic::mechanism::SinglePneumatic actuator;

  RobotHardware()
      : football_chassis(basic::chassis::new_chassis_init({
            {{
                {kFrontLeftMotorPort, vex::ratio6_1, true},
                {kBackLeftMotorPort, vex::ratio6_1, true},
            }},
            {{
                {kFrontRightMotorPort, vex::ratio6_1, false},
                {kBackRightMotorPort, vex::ratio6_1, false},
            }},
            {kCenterStrafeMotorPort, vex::ratio6_1, false},
            10,
        })),
        actuator(basic::mechanism::single_pneumatic_init({
            {brain.ThreeWirePort.A},
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

}  // namespace basic::hardware::football_robot

#endif  // BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_ROBOT_HARDWARE_H_
