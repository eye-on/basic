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

struct SteeringKinematicsConfig {
  double half_wheelbase{0.15};
  double half_track{0.15};
};

struct WheelTarget {
  double velocity_pct{0.0};
  double heading_degrees{0.0};
};

struct SteeringKinematicsResult {
  WheelTarget fr;
  WheelTarget fl;
  WheelTarget br;
  WheelTarget bl;
};

inline SteeringKinematicsResult steering_kinematics_solve(
    double vehicle_velocity_pct,
    double vehicle_heading_degrees,
    const SteeringKinematicsConfig& config) {
  const double forward_velocity = vehicle_velocity_pct;
  const double wheelbase = 2.0 * config.half_wheelbase;
  const double track_half = config.half_track;
  const double wheelbase_half = config.half_wheelbase;

  const double heading_rad = vehicle_heading_degrees * (M_PI / 180.0);
  double rotation_rate = 0.0;

  if (std::abs(forward_velocity) > 1e-9) {
    rotation_rate = forward_velocity * std::tan(heading_rad) / wheelbase;
  } else {
    rotation_rate = vehicle_heading_degrees;
  }

  auto wheel_calc = [&](double position_x, double position_y) -> WheelTarget {
    const double velocity_x = forward_velocity - rotation_rate * position_y;
    const double velocity_y = rotation_rate * position_x;
    WheelTarget target;
    target.velocity_pct = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
    target.heading_degrees = std::atan2(velocity_y, velocity_x) * (180.0 / M_PI);
    return target;
  };

  SteeringKinematicsResult result;
  result.fr = wheel_calc(wheelbase_half, track_half);
  result.fl = wheel_calc(wheelbase_half, -track_half);
  result.br = wheel_calc(-wheelbase_half, track_half);
  result.bl = wheel_calc(-wheelbase_half, -track_half);

  const double max_velocity = std::max({
      result.fr.velocity_pct, result.fl.velocity_pct,
      result.br.velocity_pct, result.bl.velocity_pct, 1.0});
  if (max_velocity > 100.0) {
    const double scale = 100.0 / max_velocity;
    result.fr.velocity_pct *= scale;
    result.fl.velocity_pct *= scale;
    result.br.velocity_pct *= scale;
    result.bl.velocity_pct *= scale;
  }

  return result;
}

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

  const SteeringKinematicsResult targets = steering_kinematics_solve(
      command.input_velocity_pct,
      command.input_heading_degrees,
      SteeringKinematicsConfig{});

  chassis.fr().control(targets.fr.velocity_pct, targets.fr.heading_degrees, command.stop_brake_type);
  chassis.fl().control(targets.fl.velocity_pct, targets.fl.heading_degrees, command.stop_brake_type);
  chassis.br().control(targets.br.velocity_pct, targets.br.heading_degrees, command.stop_brake_type);
  chassis.bl().control(targets.bl.velocity_pct, targets.bl.heading_degrees, command.stop_brake_type);
}

}//namespace basic::chassis::steering
#endif // BASIC_INCLUDE_STEERING_DRIVE_H
