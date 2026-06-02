#ifndef BASIC_INCLUDE_WHEEL_UNIT_H
#define BASIC_INCLUDE_WHEEL_UNIT_H

#include "control/kalman/calculator.hpp"
#include "control/motor_control.h"
#include "control/pid/controller.hpp"
#include "device_config.h"
#include <cmath>
#include <cstdio>

namespace basic::chassis::steering{

constexpr double kSteerGearRatio = 11.0/62.0;
constexpr double kWheelGearRatio = 1.0;
constexpr double kWheelRadiusMm = 50.0;

struct Wheel_Unit_Config {
  basic::device::MotorConfig motor1_config;
  basic::device::MotorConfig motor2_config;
};

struct Wheel_Unit {
  vex::motor motor1;
  vex::motor motor2;
  double velocity{0.0};
  double angular_velocity{0.0};
  double heading{0.0};
  const double initial_angle_m1{0.0};
  const double initial_angle_m2{0.0};
  basic::control::kalman::KalmanCalculator velocity_filter{1, 5e-2};
  basic::control::kalman::KalmanCalculator heading_filter{3e-2, 1e-1};
};

inline void wheel_unit_update(Wheel_Unit& wheel_unit) {
  const double m1_angle = wheel_unit.motor1.position(vex::deg)-wheel_unit.initial_angle_m1;
  const double m2_angle = wheel_unit.motor2.position(vex::deg)-wheel_unit.initial_angle_m2;

  const double m1_velocity = wheel_unit.motor1.velocity(vex::pct);
  const double m2_velocity = wheel_unit.motor2.velocity(vex::pct);

  const double steer_angle = (m1_angle + m2_angle) * 0.5 * kSteerGearRatio;
  const double wheel_angle = (m1_angle - m2_angle) * 0.5 * kWheelGearRatio;
  
  const double steer_velocity = (m1_velocity + m2_velocity) * kSteerGearRatio * 0.5;
  const double wheel_velocity = (m1_velocity - m2_velocity) * kWheelGearRatio * 0.5;

  printf("wheel_velocity: %.2f  ", wheel_velocity);

  wheel_unit.velocity = wheel_unit.velocity_filter.update(wheel_velocity);
  wheel_unit.heading = wheel_unit.heading_filter.update(steer_angle);
  wheel_unit.angular_velocity = steer_velocity;
}

inline void wheel_unit_control(
    Wheel_Unit& wheel_unit,
    double target_velocity_pct,
    double target_heading_degrees,
    vex::brakeType brake_type,
    basic::control::pid::Pid& velocity_pid,
    basic::control::pid::Pid& heading_pid) {
  
  wheel_unit_update(wheel_unit);

  const auto velocity_result = velocity_pid.update(target_velocity_pct, wheel_unit.velocity);
  const auto heading_result = heading_pid.update(target_heading_degrees, wheel_unit.heading);
  const double motor1_output = velocity_result.ctrl + heading_result.ctrl;
  const double motor2_output = -velocity_result.ctrl + heading_result.ctrl;

  printf("target_velocity_pct: %.2f\n", target_velocity_pct);
  
  basic::control::velocitycontrol(wheel_unit.motor1, motor1_output, vex::pct);
  basic::control::velocitycontrol(wheel_unit.motor2, motor2_output, vex::pct);
}

}//namespace basic::chassis::steering

#endif // BASIC_INCLUDE_WHEEL_UNIT_H
