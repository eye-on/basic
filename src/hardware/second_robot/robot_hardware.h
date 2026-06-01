#ifndef BASIC_SRC_HARDWARE_SECOND_ROBOT_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_SECOND_ROBOT_ROBOT_HARDWARE_H_

#include "chassis/second_chassis.h"
#include "mechanism/roller_shooter.h"
#include "vex.h"

namespace basic::hardware::second_robot {

inline constexpr int kRefreshTime = 10;
inline constexpr int kSensorLoopDelay = 50;

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{vex::PORT7};
  basic::chassis::SecondChassis second_chassis;
  basic::mechanism::RollerShooter shooter;

  RobotHardware()
      : second_chassis(basic::chassis::second_chassis_init({
            {{
                {vex::PORT1, vex::ratio6_1, true},
                {vex::PORT2, vex::ratio6_1, true},
                {vex::PORT3, vex::ratio6_1, true},
            }},
            {{
                {vex::PORT6, vex::ratio6_1, false},
                {vex::PORT13, vex::ratio6_1, false},
                {vex::PORT17, vex::ratio6_1, false},
            }},
            10,
        })),
        shooter(basic::mechanism::roller_shooter_init({
            {vex::PORT14, vex::ratio6_1, false},
            {vex::PORT15, vex::ratio6_1, true},
            {vex::PORT19, vex::ratio6_1, true},
            {brain.ThreeWirePort.F},
            {brain.ThreeWirePort.E},
            {brain.ThreeWirePort.H},
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

}  // namespace basic::hardware::second_robot

#endif
