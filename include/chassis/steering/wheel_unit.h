#ifndef BASIC_INCLUDE_WHEEL_UNIT_H
#define BASIC_INCLUDE_WHEEL_UNIT_H

#include "control/motor_control.h"
#include "control/pid/controller.hpp"
#include "device_config.h"

#include <cmath>
#include <cstdio>
#include <utility>
#include <cstddef>
#include <algorithm>

namespace basic::chassis::steering {
namespace detail {

/// 将角度规整到 [-180, 180] 范围
inline double wrap_180(double angle) {
  angle = std::fmod(angle, 360.0);
  if (angle > 180.0) angle -= 360.0;
  else if (angle < -180.0) angle += 360.0;
  return angle;
}

/// 计算从 current 到 target 的最短路径目标值
inline double shortest_path_target(double target, double current) {
  return current + wrap_180(target - current);
}

}  // namespace detail

constexpr double kSteerGearRatio = 11.0 / 62.0;
constexpr double kHalfSteerGearRatio = kSteerGearRatio * 0.5;
constexpr double kWheelGearRatio = 1.0;
constexpr double kWheelRadiusMm = 50.0;
constexpr double kWheelCircumference = 2.0 * M_PI * kWheelRadiusMm * 1e-3; // m

struct WheelUnitConfig {
  basic::device::MotorConfig motor_a;
  basic::device::MotorConfig motor_b;
  basic::control::pid::Pid::Config velocity_pid;
  basic::control::pid::Pid::Config heading_pid;
  basic::control::pid::Pid::Config angular_velocity_pid;
  first_order_adrc::Controller::Config adrc_a;
  first_order_adrc::Controller::Config adrc_b;
  double motor_max_rpm{600};  // 电机最高转速（蓝盒600，绿盒200，红盒100）
};

struct WheelUnitState {
  double velocity{0.0};
  double heading{0.0};
  double steer_velocity{0.0};
  double initial_angle_a{0.0};
  double initial_angle_b{0.0};
};

class WheelUnit {
 private:
  vex::motor motor_a_;
  vex::motor motor_b_;
  WheelUnitState state_;
  basic::control::pid::Pid velocity_pid;
  basic::control::pid::Pid heading_pid;
  basic::control::pid::Pid angular_velocity_pid;
  first_order_adrc::Controller adrc_a_;
  first_order_adrc::Controller adrc_b_;
  /// pct → m/s 转换因子（构造时预计算，避免每帧重复运算）
  /// mps = pct × factor, 其中 factor = max_rpm / 100 / 60 × 2πr
  const double pct_to_mps_factor_;

public:
  WheelUnit(vex::motor&& motor_a, vex::motor&& motor_b,
             double init_angle_a, double init_angle_b,
             const basic::control::pid::Pid::Config& vpid_cfg,
             const basic::control::pid::Pid::Config& hpid_cfg,
             const basic::control::pid::Pid::Config& avpid_cfg,
             const first_order_adrc::Controller::Config& adrc_a_cfg,
             const first_order_adrc::Controller::Config& adrc_b_cfg,
             double motor_max_rpm)
      : motor_a_(std::move(motor_a)),
        motor_b_(std::move(motor_b)),
        state_{0.0, 0.0, 0.0, init_angle_a, init_angle_b},
        velocity_pid(vpid_cfg),
        heading_pid(hpid_cfg),
        angular_velocity_pid(avpid_cfg),
        adrc_a_(adrc_a_cfg),
        adrc_b_(adrc_b_cfg),
        pct_to_mps_factor_(motor_max_rpm / 100.0 / 60.0 * kWheelCircumference) {}

  void update() {
    const double angle_a = motor_a_.position(vex::deg) - state_.initial_angle_a;
    const double angle_b = motor_b_.position(vex::deg) - state_.initial_angle_b;

    const double vel_a = motor_a_.velocity(vex::pct);
    const double vel_b = motor_b_.velocity(vex::pct);

    // 直接写入 state_，避免创建中间局部变量
    state_.heading = (angle_a + angle_b) * kHalfSteerGearRatio;
    state_.steer_velocity = (vel_a + vel_b) * kHalfSteerGearRatio;
    state_.velocity = (vel_a - vel_b) * kWheelGearRatio * 0.5;
  }

  void control(double target_velocity_mps,
               double target_heading,
               vex::brakeType brake_type) {
    update();

    // 航向误差（一次 wrap_180，两处复用：路径规划 + 复位判定）
    const double heading_error = detail::wrap_180(target_heading - state_.heading);

    // 航向角位置环
    const auto heading_result = heading_pid.update(
        state_.heading + heading_error, state_.heading);

    // 航向角速度环
    const auto steer_result = angular_velocity_pid.update(
        heading_result.ctrl, state_.steer_velocity);

    // 反馈值 pct → m/s
    const double feedback_mps = state_.velocity * pct_to_mps_factor_;

    // 轮速环
    const auto velocity_result = velocity_pid.update(target_velocity_mps, feedback_mps);

    // 复位判定（复用 heading_error，避免二次 wrap_180）
    if (std::abs(heading_error) < 0.5 && std::abs(state_.steer_velocity) < 1.0) {
      heading_pid.reset();
      angular_velocity_pid.reset();
    }
    if (std::abs(target_velocity_mps) < 0.01 && std::abs(feedback_mps) < 0.01) {
      velocity_pid.reset();
    }

    // 组合输出，超限时等比例缩放
    const double v = velocity_result.ctrl;
    const double s = steer_result.ctrl;
    const double max_abs = std::max(std::abs(v + s), std::abs(-v + s));
    const double scale = (max_abs > 100.0) ? (100.0 / max_abs) : 1.0;

    basic::control::adrc_torque_control(motor_a_, (v + s) * scale, adrc_a_);
    basic::control::adrc_torque_control(motor_b_, (-v + s) * scale, adrc_b_);
  }
};

namespace detail {
inline WheelUnit make_wheel_unit(const WheelUnitConfig& config) {
  vex::motor motor_a{config.motor_a.port, config.motor_a.gear_ratio, config.motor_a.reversed};
  vex::motor motor_b{config.motor_b.port, config.motor_b.gear_ratio, config.motor_b.reversed};
  vex::this_thread::sleep_for(10);
  const double init_angle_a = motor_a.position(vex::deg);
  const double init_angle_b = motor_b.position(vex::deg);

  return WheelUnit{
      std::move(motor_a),
      std::move(motor_b),
      init_angle_a,
      init_angle_b,
      config.velocity_pid,
      config.heading_pid,
      config.angular_velocity_pid,
      config.adrc_a,
      config.adrc_b,
      config.motor_max_rpm,
  };
}
}  // namespace detail
}  // namespace basic::chassis::steering

#endif // BASIC_INCLUDE_WHEEL_UNIT_H
