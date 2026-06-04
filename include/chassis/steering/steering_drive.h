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
  double input_velocity_pct{0.0};
  double input_heading_degrees{0.0};
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
  int deadzone{10};
};

class SteeringDrive {
public:
  explicit SteeringDrive(const SteeringDriveConfig& config)
      : fr_(detail::make_wheel_unit(config.fr)),
        fl_(detail::make_wheel_unit(config.fl)),
        br_(detail::make_wheel_unit(config.br)),
        bl_(detail::make_wheel_unit(config.bl)),
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

private:
  Wheel_Unit fr_;
  Wheel_Unit fl_;
  Wheel_Unit br_;
  Wheel_Unit bl_;
  int deadzone_;
  SteeringDriveState state_;
};//class SteeringDrive

inline SteeringDrive steering_init(const SteeringDriveConfig& config){
  printf("target,measurement,erro\n");
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
  
  chassis.fr().control(command.input_velocity_pct, command.input_heading_degrees, command.stop_brake_type);
  //chassis.fl().control(command.input_velocity_pct, command.input_heading_degrees, command.stop_brake_type);
  //chassis.br().control(command.input_velocity_pct, command.input_heading_degrees, command.stop_brake_type);
  //chassis.bl().control(command.input_velocity_pct, command.input_heading_degrees, command.stop_brake_type);
}

}//namespace basic::chassis::steering
#endif // BASIC_INCLUDE_STEERING_DRIVE_H
