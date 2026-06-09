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

constexpr double kRadToDeg = 180.0 / M_PI;

enum class ControllerAxis {
  kAxis1,
  kAxis2,
  kAxis3,
  kAxis4,
};

struct ArcadeDriveCommand {
  double vx_mps{0.0};       // m/s
  double vy_mps{0.0};       // m/s
  double omega_radps{0.0};  // rad/s
  vex::brakeType stop_brake_type{vex::coast};
};

struct SteeringDriveState {
  double vx_mps{0.0};       // m/s
  double vy_mps{0.0};       // m/s
  double omega_radps{0.0};  // rad/s
  vex::brakeType stop_brake_type{vex::coast};
};

struct SteeringKinematicsConfig {
  double half_wheelbase{0.175};   // 半轴距 (m)，即 wheelbase/2
  double half_track_width{0.175}; // 半轮距 (m)，即 track_width/2
};

struct WheelTarget {
  double velocity_mps{0.0};
  double heading_degrees{0.0};
};

struct SteeringKinematicsResult {
  WheelTarget fr;
  WheelTarget fl;
  WheelTarget br;
  WheelTarget bl;
};

/// 四轮独立转向运动学正解
/// 轮子位置以质心为原点，使用实际半轴距/半轮距（单位：m）：
///   FR(+hw,+ht)  FL(+hw,-ht)  BR(-hw,+ht)  BL(-hw,-ht)
/// 刚体运动在该点的合速度：
///   vx_i = vx_mps - omega_radps * py
///   vy_i = vy_mps + omega_radps * px
///   speed_i = sqrt(vx_i² + vy_i²)
///   angle_i = atan2(vy_i, vx_i) → deg
inline SteeringKinematicsResult steering_kinematics_solve(
    const double vx_mps, const double vy_mps, const double omega_radps,
    const SteeringKinematicsConfig& config) {

  const double hw = config.half_wheelbase;
  const double ht = config.half_track_width;

  auto wheel_calc = [=](const double px, const double py) -> WheelTarget {
    const double vx_i = vx_mps - omega_radps * py;
    const double vy_i = vy_mps + omega_radps * px;
    return {std::sqrt(vx_i * vx_i + vy_i * vy_i),
            std::atan2(vy_i, vx_i) * kRadToDeg};
  };

  return {
    wheel_calc( hw,  ht),  // 右前
    wheel_calc( hw, -ht),  // 左前
    wheel_calc(-hw,  ht),  // 右后
    wheel_calc(-hw, -ht),  // 左后
  };
}

struct SteeringDriveConfig {
  WheelUnitConfig fr;
  WheelUnitConfig fl;
  WheelUnitConfig br;
  WheelUnitConfig bl;
  int deadzone{10};
  SteeringKinematicsConfig kinematics;
};

class SteeringDrive {
public:
  explicit SteeringDrive(const SteeringDriveConfig& config)
      : fr_(detail::make_wheel_unit(config.fr)),
        fl_(detail::make_wheel_unit(config.fl)),
        br_(detail::make_wheel_unit(config.br)),
        bl_(detail::make_wheel_unit(config.bl)),
        kinematics_config_(config.kinematics),
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

  const SteeringKinematicsConfig& kinematics_config() const { return kinematics_config_; }

private:
  WheelUnit fr_;
  WheelUnit fl_;
  WheelUnit br_;
  WheelUnit bl_;
  SteeringKinematicsConfig kinematics_config_;
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
  // 加载到局部变量，避免重复访问 struct 成员
  const double vx = command.vx_mps;
  const double vy = command.vy_mps;
  const double omega = command.omega_radps;
  const vex::brakeType brake = command.stop_brake_type;

  chassis.state().vx_mps = vx;
  chassis.state().vy_mps = vy;
  chassis.state().omega_radps = omega;
  chassis.state().stop_brake_type = brake;

  const SteeringKinematicsResult targets = steering_kinematics_solve(
      vx, vy, omega, chassis.kinematics_config());

  chassis.fr().control(targets.fr.velocity_mps, targets.fr.heading_degrees, brake);
  chassis.fl().control(targets.fl.velocity_mps, targets.fl.heading_degrees, brake);
  chassis.br().control(targets.br.velocity_mps, targets.br.heading_degrees, brake);
  chassis.bl().control(targets.bl.velocity_mps, targets.bl.heading_degrees, brake);
}

}  // namespace basic::chassis::steering

#endif // BASIC_INCLUDE_STEERING_DRIVE_H
