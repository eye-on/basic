#ifndef BASIC_SRC_HARDWARE_ROBOTS_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_ROBOTS_ROBOT_HARDWARE_H_

#include "vex.h"

namespace basic::hardware::robots {

inline constexpr bool kIsBlue = false;
inline constexpr int kRefreshTime = 10;
inline constexpr int kDeadZone = 10;
inline constexpr int kSensorLoopDelay = 50;
struct RobotHardware {
  vex::motor motor_fr1{vex::PORT14, vex::ratio6_1, false};
  vex::motor motor_fr2{vex::PORT9, vex::ratio6_1, true};

  vex::motor motor_br1{vex::PORT8, vex::ratio6_1, true};
  vex::motor motor_br2{vex::PORT7, vex::ratio6_1, false};

  vex::motor motor_fl1{vex::PORT6, vex::ratio6_1, true};
  vex::motor motor_fl2{vex::PORT5, vex::ratio6_1, false};

  vex::motor motor_bl1{vex::PORT4, vex::ratio6_1, false};
  vex::motor motor_bl2{vex::PORT3, vex::ratio6_1, true};

  vex::motor middle_motor1{vex::PORT16, vex::ratio6_1, true};
  vex::motor under_motor1{vex::PORT11, vex::ratio18_1, true};
  vex::motor upper_motor1{vex::PORT19, vex::ratio6_1, true};

  vex::motor trans_motor1{vex::PORT12, vex::ratio6_1, true};
  vex::motor trans_motor2{vex::PORT1, vex::ratio6_1, true};
  vex::motor trans_motor3{vex::PORT15, vex::ratio18_1, true};

  vex::motor under_overhang_motor{vex::PORT18, vex::ratio18_1, true}; 
  vex::motor upper_overhang_motor{vex::PORT2, vex::ratio18_1, true};
  vex::motor middle_overhang_motor{vex::PORT17, vex::ratio18_1, true};

  vex::distance laser_rangefinder{vex::PORT13};
  vex::optical color_sensor{vex::PORT20};
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{vex::PORT10};

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

}  // namespace basic::hardware::robots

#endif
