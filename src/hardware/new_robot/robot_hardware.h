#ifndef BASIC_SRC_HARDWARE_NEW_ROBOT_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_NEW_ROBOT_ROBOT_HARDWARE_H_

#include "chassis/new_chassis.h"
#include "vex.h"

namespace basic::hardware::new_robot {

inline constexpr int kRefreshTime = 10;
inline constexpr int kSensorLoopDelay = 50;

inline constexpr basic::control::pid::Pid::Config kVeloPidCfg{
    0.7, 0.0, 0.0,
    -100.0, 100.0, -10.0, 10.0, 3,
    basic::control::pid::Type::kIncremental};

inline constexpr basic::control::pid::Pid::Config kHeadingPidCfg{
    1, 0.0, 0.0,
    -100, 100, -1000.0, 1000.0, 1};

inline constexpr basic::control::pid::Pid::Config kAngularVelocityPidCfg{
    0.5, 0.0, 0.0,
    -100.0, 100.0, -10.0, 10.0, 1,
    basic::control::pid::Type::kIncremental};

inline constexpr first_order_adrc::Controller::Config kAdrcCfg;

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{vex::PORT11};
  basic::chassis::NewChassis new_chassis;

  RobotHardware()
      : new_chassis(basic::chassis::new_chassis_init({
            {
                {vex::PORT12, vex::ratio18_1, true},
                {vex::PORT20, vex::ratio18_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
            },
            {
                {vex::PORT3, vex::ratio6_1, true},
                {vex::PORT4, vex::ratio6_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
            },
            {
                {vex::PORT5, vex::ratio6_1, true},
                {vex::PORT6, vex::ratio6_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
            },
            {
                {vex::PORT7, vex::ratio6_1, true},
                {vex::PORT8, vex::ratio6_1, true},
                kVeloPidCfg,
                kHeadingPidCfg,
                kAngularVelocityPidCfg,
                kAdrcCfg,
                kAdrcCfg,
            },
            1,
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
