#ifndef BASIC_INCLUDE_WHEEL_UNIT_H
#define BASIC_INCLUDE_WHEEL_UNIT_H

#include "control/kalman/calculator.hpp"
#include "control/motor_control.h"
#include "control/pid/controller.hpp"
#include "device_config.h"
#include <cmath>
#include <cstdio>
#include <utility>

namespace basic::chassis::steering{

constexpr double kSteerGearRatio = 11.0/62.0;
constexpr double kWheelGearRatio = 1.0;
constexpr double kWheelRadiusMm = 50.0;

struct Wheel_Unit_Config {
  basic::device::MotorConfig motor1_config;
  basic::device::MotorConfig motor2_config;
  basic::control::pid::Pid::Config velocity_pid_config;
  basic::control::pid::Pid::Config heading_pid_config;
  first_order_adrc::Controller::Config adrc1_config;
  first_order_adrc::Controller::Config adrc2_config;
};

class Wheel_Unit {
 private:
  vex::motor motor1;
  vex::motor motor2;
  double velocity{0.0};
  double heading{0.0};
  const double initial_angle_m1;
  const double initial_angle_m2;
  basic::control::kalman::KalmanCalculator m1_angle_filter{0.005, 1e-4};
  basic::control::kalman::KalmanCalculator m2_angle_filter{0.005, 1e-4};
  basic::control::kalman::KalmanCalculator m1_velocity_filter{0.902649, 0.097258, 1.2703, 0.0259, 0.9803, 100};
  basic::control::kalman::KalmanCalculator m2_velocity_filter{0.902649, 0.097258, 1.2703, 0.0259, 0.9803, 100};
  basic::control::pid::Pid velocity_pid;
  basic::control::pid::Pid heading_pid;
  first_order_adrc::Controller adrc1;
  first_order_adrc::Controller adrc2;

public:
  Wheel_Unit(vex::motor&& m1, vex::motor&& m2,
             double init_m1, double init_m2,
             const basic::control::pid::Pid::Config& vpid_cfg,
             const basic::control::pid::Pid::Config& hpid_cfg,
             const first_order_adrc::Controller::Config& a1_cfg,
             const first_order_adrc::Controller::Config& a2_cfg)
      : motor1(std::move(m1)),
        motor2(std::move(m2)),
        initial_angle_m1(init_m1),
        initial_angle_m2(init_m2),
        velocity_pid(vpid_cfg),
        heading_pid(hpid_cfg),
        adrc1(a1_cfg),
        adrc2(a2_cfg) {}

  void update() {
    const double m1_angle_raw = motor1.position(vex::deg) - initial_angle_m1;
    const double m2_angle_raw = motor2.position(vex::deg) - initial_angle_m2;

    const double m1_velocity_raw = motor1.velocity(vex::pct);
    const double m2_velocity_raw = motor2.velocity(vex::pct);

    const double m1_angle = m1_angle_filter.update(m1_angle_raw);
    const double m2_angle = m2_angle_filter.update(m2_angle_raw);
    const double m1_velocity = m1_velocity_filter.update(m1_velocity_raw);
    const double m2_velocity = m2_velocity_filter.update(m2_velocity_raw);

    const double steer_angle = (m1_angle + m2_angle) * 0.5 * kSteerGearRatio;

    const double wheel_velocity = (m1_velocity - m2_velocity) * kWheelGearRatio * 0.5;

    velocity = wheel_velocity;
    heading = steer_angle;
  }

  void control(double target_velocity_pct,
               double target_heading_degrees,
               vex::brakeType brake_type) {
    update();

    const auto velocity_result = velocity_pid.update(target_velocity_pct, velocity);
    const auto heading_result = heading_pid.update(target_heading_degrees, heading);
    const double motor1_output = velocity_result.ctrl + heading_result.ctrl;
    const double motor2_output = -velocity_result.ctrl + heading_result.ctrl;

    //const double motor1_output = target_velocity_pct + target_heading_degrees;
    //const double motor2_output = -target_velocity_pct + target_heading_degrees;

    printf("%.2f,",motor1_output);

    //basic::control::adrc_velocity_control(motor1, motor1_output, adrc1);
    //basic::control::adrc_velocity_control(motor2, motor2_output, adrc2);
    basic::control::velocitycontrol(motor1, motor1_output);
    basic::control::velocitycontrol(motor2, motor2_output);
  }
};

namespace detail {

inline Wheel_Unit make_wheel_unit(const Wheel_Unit_Config& config) {
  vex::motor motor1{config.motor1_config.port, config.motor1_config.gear_ratio, config.motor1_config.reversed};
  vex::motor motor2{config.motor2_config.port, config.motor2_config.gear_ratio, config.motor2_config.reversed};
  vex::this_thread::sleep_for(10);
  const double initial_m1 = motor1.position(vex::deg);
  const double initial_m2 = motor2.position(vex::deg);

  return Wheel_Unit{
      std::move(motor1),
      std::move(motor2),
      initial_m1,
      initial_m2,
      config.velocity_pid_config,
      config.heading_pid_config,
      config.adrc1_config,
      config.adrc2_config,
  };
}

}  // namespace detail

}//namespace basic::chassis::steering

#endif // BASIC_INCLUDE_WHEEL_UNIT_H
