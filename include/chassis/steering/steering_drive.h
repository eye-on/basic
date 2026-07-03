#ifndef BASIC_INCLUDE_STEERING_DRIVE_H
#define BASIC_INCLUDE_STEERING_DRIVE_H

#include "control/motor_control.h"
#include "control/pid/controller.hpp"
#include "device_config.h"
#include "hardware/shared/state_types.h"
#include "wheel_unit.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <tuple>
#include <utility>

namespace basic::chassis::steering {

constexpr double kRadToDeg = 180.0 / M_PI;
/// 速度缩放因子：运动学输出 ≤100pct，实际需更大驱动力克服地面摩擦（经验值）
constexpr double kVelocityScaleFactor = 2.0;
/// square_to_circle 归一化常量：0.5 / 100²，预计算避免每帧两次除法
constexpr double kSqToCircleNorm = 0.5 / (100.0 * 100.0);  // = 0.00005

enum class ControllerAxis {
  kAxis1,
  kAxis2,
  kAxis3,
  kAxis4,
};

struct ArcadeDriveCommand {
  double vx_pct{0.0};       // pct
  double vy_pct{0.0};       // pct
  double omega_pct{0.0};  // pct
  vex::brakeType stop_brake_type{vex::coast};
};

struct SteeringDriveState {
  double vx_pct{0.0};       // pct
  double vy_pct{0.0};       // pct
  double omega_pct{0.0};  // pct
  vex::brakeType stop_brake_type{vex::coast};
};

struct SteeringKinematicsConfig {
  /// 半轴距 / 回转半径，无因次比值。纯旋转 ω=100pct 时各轮速度 = 100pct
  double half_wheelbase_ratio{0.707};   // hw / √(hw²+ht²), ω=100时各轮 speed=100
  /// 半轮距 / 回转半径，无因次比值
  double half_track_width_ratio{0.707}; // ht / √(hw²+ht²)

  /// 从物理尺寸构造（半轴距, 半轮距，单位一致即可）
  /// 例如 from_dimensions(120.0, 100.0) → {0.768, 0.640}
  static inline SteeringKinematicsConfig from_dimensions(
      double half_wheelbase, double half_track) {
    const double r = std::sqrt(half_wheelbase * half_wheelbase +
                               half_track * half_track);
    if (r == 0.0) return {0.707, 0.707};  // 防御除零
    return {half_wheelbase / r, half_track / r};
  }
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

/// 矩形→圆映射（FG mapping），消除摇杆对角线超速
/// 输入/输出均在 pct 域 [-100, 100]，映射是齐次的无需归一化
inline std::pair<double, double> square_to_circle(double x, double y) {
  const double u = x * std::sqrt(1.0 - kSqToCircleNorm * y * y);
  const double v = y * std::sqrt(1.0 - kSqToCircleNorm * x * x);
  return {u, v};
}

/// 四轮独立转向运动学正解（纯 pct，无物理量）
/// 输入 vx_pct、vy_pct、omega_pct 均为摇杆 pct 值 [-100, 100]
/// 配置 ratio = 半轴距/回转半径（无因次）
/// 输出各轮速度 pct 和航向 °
inline SteeringKinematicsResult steering_kinematics_solve(
    const double vx_pct, const double vy_pct, const double omega_pct,
    const SteeringKinematicsConfig& config) {

  const double hw = config.half_wheelbase_ratio;
  const double ht = config.half_track_width_ratio;

  auto wheel_calc = [=](const double px, const double py) -> WheelTarget {
    const double vx_i = vx_pct - omega_pct * py;
    const double vy_i = vy_pct + omega_pct * px;
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

  /// 四轮机械标零
  void calibrate_zero() {
    fr_.calibrate_zero();
    fl_.calibrate_zero();
    br_.calibrate_zero();
    bl_.calibrate_zero();
  }

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
  if (axis == ControllerAxis::kAxis1) return input.axis1;
  if (axis == ControllerAxis::kAxis2) return input.axis2;
  if (axis == ControllerAxis::kAxis3) return input.axis3;
  return input.axis4;
}

/// 摇杆死区（pct 域）
inline double apply_deadzone(double pct, int threshold) {
  return (std::abs(pct) < threshold) ? 0.0 : pct;
}

inline void steering_update(SteeringDrive& chassis, const ArcadeDriveCommand& command) {
  const vex::brakeType brake = command.stop_brake_type;
  const int dz = chassis.deadzone();

  // 摇杆死区
  const double vx = apply_deadzone(command.vx_pct, dz);
  const double vy = apply_deadzone(command.vy_pct, dz);
  const double omega = apply_deadzone(command.omega_pct, dz);

  // 矩形→圆映射（vx 或 vy 为零时映射无效果，跳过 sqrt）
  double vx_mapped, vy_mapped;
  if (vx == 0.0 || vy == 0.0) {
    vx_mapped = vx;
    vy_mapped = vy;
  } else {
    std::tie(vx_mapped, vy_mapped) = square_to_circle(vx, vy);
  }

  chassis.state().vx_pct = vx_mapped;
  chassis.state().vy_pct = vy_mapped;
  chassis.state().omega_pct = omega;
  chassis.state().stop_brake_type = brake;

  const SteeringKinematicsResult targets = steering_kinematics_solve(
      vx_mapped, vy_mapped, omega, chassis.kinematics_config());

  // 速度缩放补偿（经验值）
  chassis.fr().control(kVelocityScaleFactor * targets.fr.velocity_pct, targets.fr.heading_degrees, brake);
  chassis.fl().control(kVelocityScaleFactor * targets.fl.velocity_pct, targets.fl.heading_degrees, brake);
  chassis.br().control(kVelocityScaleFactor * targets.br.velocity_pct, targets.br.heading_degrees, brake);
  chassis.bl().control(kVelocityScaleFactor * targets.bl.velocity_pct, targets.bl.heading_degrees, brake);
}

/// 舵轮底盘机械标零
inline void steering_calibrate_zero(SteeringDrive& chassis) {
  chassis.calibrate_zero();
}

}  // namespace basic::chassis::steering

#endif // BASIC_INCLUDE_STEERING_DRIVE_H
