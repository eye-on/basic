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

namespace basic::chassis::steering {

enum class ControllerAxis {
  kAxis1,
  kAxis2,
  kAxis3,
  kAxis4,
};

struct ArcadeDriveCommand {
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
  vex::brakeType stop_brake_type{vex::coast};
};

struct SteeringDriveState {
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
  vex::brakeType stop_brake_type{vex::coast};
};

struct SteeringKinematicsConfig {
  double turn_gain{3.0};  
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

/// 四轮独立转向运动学正解（无量纲模型）
/// 轮子位置简化为距离质心 ±1 的正方形：
///   FR(+1,+1)  FL(+1,-1)  BR(-1,+1)  BL(-1,-1)
/// 刚体运动在该点的合速度：
///   vx_i = vx - omega * py
///   vy_i = vy + omega * px
///   speed_i = sqrt(vx_i² + vy_i²)
///   angle_i = atan2(vy_i, vx_i) → deg
inline SteeringKinematicsResult steering_kinematics_solve(
    double vx, double vy, double omega,
    const SteeringKinematicsConfig& config) {

  auto wheel_calc = [&](double px, double py) -> WheelTarget {
    const double vx_i = vx - omega * py;
    const double vy_i = vy + omega * px;
    return {std::sqrt(vx_i * vx_i + vy_i * vy_i),
            std::atan2(vy_i, vx_i) * (180.0 / M_PI)};
  };

  SteeringKinematicsResult result;
  result.fr = wheel_calc( 1,  1);  // 右前
  result.fl = wheel_calc( 1, -1);  // 左前
  result.br = wheel_calc(-1,  1);  // 右后
  result.bl = wheel_calc(-1, -1);  // 左后

  return result;
}

struct SteeringDriveConfig {
  WheelUnitConfig fr;
  WheelUnitConfig fl;
  WheelUnitConfig br;
  WheelUnitConfig bl;
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

  WheelUnit& fr() { return fr_; }
  const WheelUnit& fr() const { return fr_; }
  WheelUnit& fl() { return fl_; }
  const WheelUnit& fl() const { return fl_; }
  WheelUnit& br() { return br_; }
  const WheelUnit& br() const { return br_; }
  WheelUnit& bl() { return bl_; }
  const WheelUnit& bl() const { return bl_; }

  int deadzone() const { return deadzone_; }

private:
  WheelUnit fr_;
  WheelUnit fl_;
  WheelUnit br_;
  WheelUnit bl_;
  int deadzone_;
  SteeringDriveState state_;
};  // class SteeringDrive

inline SteeringDrive steering_init(const SteeringDriveConfig& config) {
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
  chassis.state().vx = command.vx;
  chassis.state().vy = command.vy;
  chassis.state().omega = command.omega;
  chassis.state().stop_brake_type = command.stop_brake_type;

  const SteeringKinematicsResult targets = steering_kinematics_solve(
      command.vx,
      command.vy,
      command.omega,
      SteeringKinematicsConfig{});

  chassis.fr().control(targets.fr.velocity_pct, targets.fr.heading_degrees, command.stop_brake_type);
  chassis.fl().control(targets.fl.velocity_pct, targets.fl.heading_degrees, command.stop_brake_type);
  chassis.br().control(targets.br.velocity_pct, targets.br.heading_degrees, command.stop_brake_type);
  chassis.bl().control(targets.bl.velocity_pct, targets.bl.heading_degrees, command.stop_brake_type);
}

}  // namespace basic::chassis::steering

#endif // BASIC_INCLUDE_STEERING_DRIVE_H
