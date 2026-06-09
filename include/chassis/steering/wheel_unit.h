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

inline double clamp_value(double value, double min_value, double max_value) {
  return std::min(std::max(value, min_value), max_value);
}

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

constexpr double kSteerGearRatio = 11.0/62.0;
constexpr double kWheelGearRatio = 1.0;
constexpr double kWheelRadiusMm = 50.0;

struct WheelUnitConfig {
  basic::device::MotorConfig motor_a;
  basic::device::MotorConfig motor_b;
  basic::control::pid::Pid::Config velocity_pid;
  basic::control::pid::Pid::Config heading_pid;
  basic::control::pid::Pid::Config angular_velocity_pid;
  first_order_adrc::Controller::Config adrc_a;
  first_order_adrc::Controller::Config adrc_b;
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

public:
  WheelUnit(vex::motor&& motor_a, vex::motor&& motor_b,
             double init_angle_a, double init_angle_b,
             const basic::control::pid::Pid::Config& vpid_cfg,
             const basic::control::pid::Pid::Config& hpid_cfg,
             const basic::control::pid::Pid::Config& avpid_cfg,
             const first_order_adrc::Controller::Config& adrc_a_cfg,
             const first_order_adrc::Controller::Config& adrc_b_cfg)
      : motor_a_(std::move(motor_a)),
        motor_b_(std::move(motor_b)),
        state_{0.0, 0.0, 0.0, init_angle_a, init_angle_b},
        velocity_pid(vpid_cfg),
        heading_pid(hpid_cfg),
        angular_velocity_pid(avpid_cfg),
        adrc_a_(adrc_a_cfg),
        adrc_b_(adrc_b_cfg) {}

  void update() {
    const double angle_a = motor_a_.position(vex::deg) - state_.initial_angle_a;
    const double angle_b = motor_b_.position(vex::deg) - state_.initial_angle_b;

    const double vel_a = motor_a_.velocity(vex::pct);
    const double vel_b = motor_b_.velocity(vex::pct);

    const double steer_angle = (angle_a + angle_b) * 0.5 * kSteerGearRatio;

    const double steer_velocity = (vel_a + vel_b) * 0.5 * kSteerGearRatio;
    const double wheel_velocity = (vel_a - vel_b) * kWheelGearRatio * 0.5;

    state_.velocity = wheel_velocity;
    state_.heading = steer_angle;
    state_.steer_velocity = steer_velocity;
  }

  void control(double target_velocity_pct,
               double target_heading,
               vex::brakeType brake_type) {
    update();

    // 航向角位置环（位置式 PID，直接输出）
    const double planned_heading = detail::shortest_path_target(target_heading, state_.heading);
    const auto heading_result = heading_pid.update(planned_heading, state_.heading);

    // 航向角速度环（增量式 PID，.ctrl 已是内部累加后的绝对值）
    const auto steer_result = angular_velocity_pid.update(heading_result.ctrl, state_.steer_velocity);

    // 轮速环（增量式 PID，.ctrl 已是内部累加后的绝对值）
    const auto velocity_result = velocity_pid.update(target_velocity_pct, state_.velocity);

    // 到达目标时重置 PID 内部累加量，防止积分饱和
    if (std::abs(detail::wrap_180(target_heading - state_.heading)) < 0.5
        && std::abs(state_.steer_velocity) < 1.0) {
      heading_pid.reset();
      angular_velocity_pid.reset();
    }
    if (std::abs(target_velocity_pct) < 0.5 && std::abs(state_.velocity) < 1.0) {
      velocity_pid.reset();
    }

    // 组合输出并钳位到 ±100%，防止电机过驱
    const double motor_a_output = detail::clamp_value(
        velocity_result.ctrl + steer_result.ctrl, -100.0, 100.0);
    const double motor_b_output = detail::clamp_value(
        -velocity_result.ctrl + steer_result.ctrl, -100.0, 100.0);

    basic::control::adrc_torque_control(motor_a_, motor_a_output, adrc_a_);
    basic::control::adrc_torque_control(motor_b_, motor_b_output, adrc_b_);
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
  };
}
}  // namespace detail
}  // namespace basic::chassis::steering

#endif // BASIC_INCLUDE_WHEEL_UNIT_H
