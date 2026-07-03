#ifndef BASIC_SRC_HARDWARE_NEW_ROBOT_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_NEW_ROBOT_ROBOT_HARDWARE_H_

#include "chassis/new_chassis.h"
#include "vex.h"

namespace basic::hardware::new_robot {

inline constexpr int kRefreshTime = 10;
inline constexpr int kSensorLoopDelay = 50;

inline constexpr basic::control::pid::Pid::Config kVeloPidCfg{
    200.0, 0.0, 0.0,
    -200.0, 200.0, -200.0, 200.0, 0.05};   

//{30, 0.5, 0.0,-200.0, 200.0, -50.0, 50.0, 0.05}

inline constexpr basic::control::pid::Pid::Config kHeadingPidCfg{
    1.0, 0.0, 0.0,
    -25.0, 25.0, -1000.0, 1000.0, 0.5};  

inline constexpr basic::control::pid::Pid::Config kAngularVelocityPidCfg{
    6.0, 0.0, 0.0,
    -100.0, 100.0, -10.0, 10.0, 0.5};   
inline constexpr first_order_adrc::Controller::Config kAdrcCfg;

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{vex::PORT11};
  basic::chassis::NewChassis new_chassis;

  RobotHardware() //fr fl br bl
      : new_chassis(basic::chassis::new_chassis_init({
            {
                {vex::PORT3, vex::ratio18_1, true},
                {vex::PORT2, vex::ratio18_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
                200
            },
            {
                {vex::PORT12, vex::ratio18_1, true},
                {vex::PORT13, vex::ratio18_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
                200
            },
            {
                {vex::PORT10, vex::ratio18_1, true},
                {vex::PORT9, vex::ratio18_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
                200
            },
            {
                {vex::PORT19, vex::ratio18_1, true},
                {vex::PORT18, vex::ratio18_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
                200
            },
            5,
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

}  // namespace basic::hardware::new_robot

#endif  // BASIC_SRC_HARDWARE_NEW_ROBOT_ROBOT_HARDWARE_H_
