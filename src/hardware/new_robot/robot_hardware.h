#ifndef BASIC_SRC_HARDWARE_NEW_ROBOT_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_NEW_ROBOT_ROBOT_HARDWARE_H_

#include "chassis/new_chassis.h"
#include "vex.h"

namespace basic::hardware::new_robot {

inline constexpr int kRefreshTime = 10;
inline constexpr int kSensorLoopDelay = 50;

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{vex::PORT11};
  basic::chassis::NewChassis new_chassis;

  RobotHardware()
      : new_chassis(basic::chassis::new_chassis_init({
            // 右前轮组 (fr): motor1, motor2
            {
                {vex::PORT12, vex::ratio6_1, true}, // motor1_config
                {vex::PORT20, vex::ratio6_1, false} // motor2_config
            },
            // 左前轮组 (fl): motor1, motor2
            {
                {vex::PORT3, vex::ratio6_1, false}, // motor1_config
                {vex::PORT4, vex::ratio6_1, true}  // motor2_config
            },
            // 右后轮组 (br): motor1, motor2
            {
                {vex::PORT5, vex::ratio6_1, true}, // motor1_config
                {vex::PORT6, vex::ratio6_1, false} // motor2_config
            },
            // 左后轮组 (bl): motor1, motor2
            {
                {vex::PORT7, vex::ratio6_1, false}, // motor1_config
                {vex::PORT8, vex::ratio6_1, true}  // motor2_config
            },
            // 速度PID配置
            {
                basic::control::pid::Mode::kLinear, // mode
                2.0,   // kp
                0.5,   // ki
                0.1,   // kd
                1.0,   // log_gain
                -100.0, // out_min
                100.0,  // out_max
                -1000.0, // i_term_min
                1000.0,  // i_term_max
                5.0,    // deadzone
            },
            // 航向PID配置
            {
                basic::control::pid::Mode::kLinear, // mode
                2.0,   // kp
                0.5,   // ki
                0.1,   // kd
                1.0,   // log_gain
                -180.0, // out_min
                180.0,  // out_max
                -1000.0, // i_term_min
                1000.0,  // i_term_max
                1.0,    // deadzone
            },
            10, // deadzone
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
