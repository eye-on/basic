#ifndef BASIC_INCLUDE_WHEEL_UNIT_H
#define BASIC_INCLUDE_WHEEL_UNIT_H

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
  double initial_angle_m1{0.0};
  double initial_angle_m2{0.0};
};

inline void wheel_unit_update(Wheel_Unit& wheel_unit) {
  const double m1_angle = basic::control::get_position(wheel_unit.motor1, vex::deg);
  const double m2_angle = basic::control::get_position(wheel_unit.motor2, vex::deg);
  const double m1_velocity = basic::control::get_velocity(wheel_unit.motor1, vex::pct);
  const double m2_velocity = basic::control::get_velocity(wheel_unit.motor2, vex::pct);

  const double steer_angle = (std::abs(m1_angle) - std::abs(m2_angle)) * kSteerGearRatio;
  const double wheel_angle = (m1_angle - steer_angle) * kWheelGearRatio;
  
  const double steer_velocity = (std::abs(m1_velocity) - std::abs(m2_velocity)) * kSteerGearRatio;
  const double wheel_velocity = (m1_velocity - steer_velocity) * kWheelGearRatio;

  

  wheel_unit.velocity = wheel_velocity;
  wheel_unit.heading = steer_angle;
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

  basic::control::velocitycontrol(wheel_unit.motor1, motor1_output, vex::pct);
  basic::control::velocitycontrol(wheel_unit.motor2, motor2_output, vex::pct);
}

}//namespace basic::chassis::steering

#endif // BASIC_INCLUDE_WHEEL_UNIT_H
