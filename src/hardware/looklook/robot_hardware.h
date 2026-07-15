#ifndef BASIC_SRC_HARDWARE_LOOKLOOK_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_LOOKLOOK_ROBOT_HARDWARE_H_

#include "chassis/h_chassis.h"
#include "mechanism/linear_lift.h"
#include "vex.h"

namespace basic::hardware::looklook {

inline constexpr int kRefreshTime = 10;

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{vex::PORT11};
  basic::chassis::HChassis h_chassis;
  basic::mechanism::LinearLift lift;

  RobotHardware()
      : h_chassis(basic::chassis::h_chassis_init({
            {{
                {vex::PORT1, vex::ratio6_1, true},
                {vex::PORT2, vex::ratio6_1, true},
            }},
            {{
                {vex::PORT3, vex::ratio6_1, false},
                {vex::PORT4, vex::ratio6_1, false},
            }},
            {vex::PORT5, vex::ratio6_1, false},
            10,
        })),
        lift(basic::mechanism::linear_lift_init({
            {vex::PORT6, vex::ratio18_1, true},
            {vex::PORT7, vex::ratio18_1, true},
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
