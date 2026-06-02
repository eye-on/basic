#ifndef BASIC_INCLUDE_STEERING_DRIVE_H
#define BASIC_INCLUDE_STEERING_DRIVE_H

#include "control/motor_control.h"
#include "control/pid/controller.hpp"
#include "device_config.h"
#include "hardware/shared/state_types.h"
#include "wheel_unit.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>

namespace basic::chassis::steering{

enum class ControllerAxis {
  kAxis1,
  kAxis2,
  kAxis3,
  kAxis4,
};

struct ArcadeDriveCommand {
  int input_velocity_pct{0};
  int input_heading_degrees{0};
  vex::brakeType stop_brake_type{vex::coast};
};

struct SteeringDriveState {
  double target_velocity_pct{0.0};
  double target_heading_degrees{0.0};
  double measured_velocity_pct{0.0};
  double measured_heading_degrees{0.0};
  vex::brakeType stop_brake_type{vex::coast};
};

struct SteeringDriveConfig {
  Wheel_Unit_Config fr;
  Wheel_Unit_Config fl;
  Wheel_Unit_Config br;
  Wheel_Unit_Config bl;
  basic::control::pid::Pid::Config velocity_pid_config;
  basic::control::pid::Pid::Config heading_pid_config;
  int deadzone{10};
};

namespace detail {

inline Wheel_Unit make_wheel_unit(const Wheel_Unit_Config& wheel_unit_config){
  vex::motor motor1{wheel_unit_config.motor1_config.port, wheel_unit_config.motor1_config.gear_ratio, wheel_unit_config.motor1_config.reversed};
  vex::motor motor2{wheel_unit_config.motor2_config.port, wheel_unit_config.motor2_config.gear_ratio, wheel_unit_config.motor2_config.reversed};
  motor1.resetPosition();
  motor2.resetPosition();
  const double initial_m1 = basic::control::get_position(motor1, vex::deg);
  const double initial_m2 = basic::control::get_position(motor2, vex::deg);
  
  
  return Wheel_Unit{
      std::move(motor1),
      std::move(motor2),
      0.0,
      0.0,
      0.0,
      initial_m1,
      initial_m2,
  };
}

inline double shape_input(double input_pct) {
  const bool negative = input_pct < 0.0;
  const double normalized = std::abs(input_pct) * 0.01;
  const double shaped = normalized * normalized * (3.0 - 2.0 * normalized) * 100.0;
  return negative ? -shaped : shaped;
}

}//namespace detail

class SteeringDrive {
public:
  explicit SteeringDrive(const SteeringDriveConfig& config)
      : fr_(detail::make_wheel_unit(config.fr)),
        fl_(detail::make_wheel_unit(config.fl)),
        br_(detail::make_wheel_unit(config.br)),
        bl_(detail::make_wheel_unit(config.bl)),
        velocity_pid_(config.velocity_pid_config),
        heading_pid_(config.heading_pid_config),
        deadzone_(config.deadzone) {
  }

  SteeringDriveState& state() {
    return state_;
  }

  const SteeringDriveState& state() const {
    return state_;
  }

  Wheel_Unit& fr() { return fr_; }
  const Wheel_Unit& fr() const { return fr_; }
  Wheel_Unit& fl() { return fl_; }
  const Wheel_Unit& fl() const { return fl_; }
  Wheel_Unit& br() { return br_; }
  const Wheel_Unit& br() const { return br_; }
  Wheel_Unit& bl() { return bl_; }
  const Wheel_Unit& bl() const { return bl_; }

  basic::control::pid::Pid& velocity_pid() { return velocity_pid_; }
  const basic::control::pid::Pid& velocity_pid() const { return velocity_pid_; }
  basic::control::pid::Pid& heading_pid() { return heading_pid_; }
  const basic::control::pid::Pid& heading_pid() const { return heading_pid_; }

private:
  Wheel_Unit fr_;
  Wheel_Unit fl_;
  Wheel_Unit br_;
  Wheel_Unit bl_;
  basic::control::pid::Pid velocity_pid_;
  basic::control::pid::Pid heading_pid_;
  int deadzone_;
  SteeringDriveState state_;
};//class SteeringDrive

inline SteeringDrive steering_init(const SteeringDriveConfig& config){
  return SteeringDrive(config);
}

inline int controller_axis_value(
    const basic::hardware::shared::ControllerInputState& input,
    ControllerAxis axis) {
  switch (axis) {
    case ControllerAxis::kAxis1:
      return input.axis1;
    case ControllerAxis::kAxis2:
      return input.axis2;
    case ControllerAxis::kAxis3:
      return input.axis3;
    case ControllerAxis::kAxis4:
    default:
      return input.axis4;
  }
}

inline void steering_update(SteeringDrive& chassis, const ArcadeDriveCommand& command) {
  chassis.state().target_velocity_pct = command.input_velocity_pct;
  chassis.state().target_heading_degrees = command.input_heading_degrees;
  chassis.state().stop_brake_type = command.stop_brake_type;
  
  wheel_unit_control(chassis.fr(), command.input_velocity_pct, command.input_heading_degrees, command.stop_brake_type, chassis.velocity_pid(), chassis.heading_pid());
  wheel_unit_control(chassis.fl(), command.input_velocity_pct, command.input_heading_degrees, command.stop_brake_type, chassis.velocity_pid(), chassis.heading_pid());
  wheel_unit_control(chassis.br(), command.input_velocity_pct, command.input_heading_degrees, command.stop_brake_type, chassis.velocity_pid(), chassis.heading_pid());
  wheel_unit_control(chassis.bl(), command.input_velocity_pct, command.input_heading_degrees, command.stop_brake_type, chassis.velocity_pid(), chassis.heading_pid());
}

}//namespace basic::chassis::steering
#endif // BASIC_INCLUDE_STEERING_DRIVE_H
