#ifndef BASIC_SRC_HARDWARE_BASIC_ROBOT_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_BASIC_ROBOT_ROBOT_HARDWARE_H_

#include "chassis/old_chassis.h"
#include "mechanism/indexed_intake.h"
#include "vex.h"

namespace basic::hardware::basic_robot {

inline constexpr bool kIsBlue = false;
inline constexpr int kRefreshTime = 10;
inline constexpr int kSensorLoopDelay = 50;

struct RobotHardware {
  vex::distance laser_rangefinder{vex::PORT13};
  vex::optical color_sensor{vex::PORT17};
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{vex::PORT11};
  basic::chassis::OldChassis old_chassis;
  basic::mechanism::IndexedIntake intake;

  RobotHardware()
      : old_chassis(basic::chassis::old_chassis_init({
            {{
                {vex::PORT1, vex::ratio6_1, true},
                {vex::PORT2, vex::ratio6_1, false},
                {vex::PORT3, vex::ratio6_1, false},
                {vex::PORT5, vex::ratio6_1, true},
            }},
            {{
                {vex::PORT9, vex::ratio6_1, true},
                {vex::PORT8, vex::ratio6_1, false},
                {vex::PORT6, vex::ratio6_1, true},
                {vex::PORT7, vex::ratio6_1, false},
            }},
            10,
        })),
        intake(basic::mechanism::indexed_intake_init({
            {vex::PORT19, vex::ratio6_1, true},
            {vex::PORT12, vex::ratio6_1, true},
            {vex::PORT18, vex::ratio18_1, true},
            {vex::PORT14, vex::ratio6_1, true},
            {vex::PORT4, vex::ratio18_1, true},
            {vex::PORT16, vex::ratio6_1, true},
            {vex::PORT10, vex::ratio6_1, true},
            {vex::PORT20, vex::ratio6_1, true},
            {vex::PORT17, vex::ratio6_1, true},
            {vex::PORT15, vex::ratio6_1, true},
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

}  // namespace basic::hardware::basic_robot

#endif
