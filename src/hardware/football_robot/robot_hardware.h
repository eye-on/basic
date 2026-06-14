#ifndef BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_ROBOT_HARDWARE_H_

#include "chassis/h_chassis.h"
#include "mechanism/pneumatic_motor_actuator.h"
#include "vex.h"

namespace basic::hardware::football_robot {

inline constexpr int kRefreshTime = 10;
inline constexpr double kDriveOutputLimitPct = 50.0;
inline const int kFrontLeftMotorPort = vex::PORT3;
inline const int kBackLeftMotorPort = vex::PORT7;
inline const int kFrontRightMotorPort = vex::PORT12;
inline const int kBackRightMotorPort = vex::PORT5;
inline const int kCenterStrafeMotorPort = vex::PORT18;
inline const int kActuatorMotorPort = vex::PORT4;
inline constexpr double kActuatorMotorAngleADeg = 4.0;
inline constexpr double kActuatorMotorAngleBDeg = 132.0;
inline constexpr double kActuatorMotorAutoSpeedPct = 25.0;
inline constexpr double kActuatorMotorStateToleranceDeg = 5.0;

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{vex::PORT11};
  basic::chassis::HChassis football_chassis;
  basic::mechanism::PneumaticMotorActuator actuator;

  RobotHardware()
      : football_chassis(basic::chassis::h_chassis_init({
            {{
                {kFrontLeftMotorPort, vex::ratio18_1, true},
                {kBackLeftMotorPort, vex::ratio18_1, true},
            }},
            {{
                {kFrontRightMotorPort, vex::ratio18_1, false},
                {kBackRightMotorPort, vex::ratio18_1, false},
            }},
            {kCenterStrafeMotorPort, vex::ratio18_1, false},
            10,
        })),
        actuator(basic::mechanism::pneumatic_motor_actuator_init({
            {{brain.ThreeWirePort.A}},
            {kActuatorMotorPort, vex::ratio18_1, false},
            kActuatorMotorAngleADeg,
            kActuatorMotorAngleBDeg,
            kActuatorMotorAutoSpeedPct,
            kActuatorMotorStateToleranceDeg,
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
