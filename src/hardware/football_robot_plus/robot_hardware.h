#ifndef BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_ROBOT_HARDWARE_H_

#include "chassis/x_chassis.h"
#include "identify/vision_sensor.h"
#include "vex.h"

namespace basic::hardware::football_robot_plus {

inline constexpr int kRefreshTime = 10;
inline constexpr double kDriveOutputLimitPct = 50.0;
inline const int kFrontLeftMotorPort = vex::PORT3;
inline const int kBackLeftMotorPort = vex::PORT7;
inline const int kFrontRightMotorPort = vex::PORT12;
inline const int kBackRightMotorPort = vex::PORT18;
inline const int kVisionSensorPort = vex::PORT1;

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{vex::PORT11};
  basic::identify::VisionSensorIdentifier vision_identifier;
  basic::chassis::XChassis football_chassis;

  RobotHardware()
      : vision_identifier(kVisionSensorPort),
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
            10,
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
