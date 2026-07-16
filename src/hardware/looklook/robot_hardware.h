#ifndef BASIC_SRC_HARDWARE_LOOKLOOK_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_LOOKLOOK_ROBOT_HARDWARE_H_

#include "chassis/x_chassis.h"
#include "mechanism/linear_lift.h"
#include "vex.h"

namespace basic::hardware::looklook {

inline constexpr int kRefreshTime = 10;
inline constexpr basic::control::pid::Pid::Config kDefaultPid{};

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{vex::PORT11};
  basic::chassis::XChassis x_chassis;
  basic::mechanism::LinearLift lift;

  RobotHardware()
      : x_chassis(basic::chassis::x_chassis_init({
            {{ {vex::PORT1, vex::ratio6_1, true} }},   // 左前 FL
            {{ {vex::PORT2, vex::ratio6_1, false} }},  // 右前 FR
            {{ {vex::PORT3, vex::ratio6_1, true} }},   // 左后 BL
            {{ {vex::PORT4, vex::ratio6_1, false} }},  // 右后 BR
            10,
            {{kDefaultPid}}, {{kDefaultPid}}, {{kDefaultPid}}, {{kDefaultPid}},
        })),
        lift(basic::mechanism::linear_lift_init({
            {vex::PORT5, vex::ratio6_1, true},
            {vex::PORT6, vex::ratio6_1, true},
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
