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

/// 将角度差映射到 [-90, 90]，±180° 等价于 0°
/// 用于舵轮航向规划：轮组 0° 和 180° 驱动效果相同（仅轮速反向）
inline double wrap_90(double diff) {
  diff = wrap_180(diff);
  if (diff > 90.0) diff -= 180.0;
  else if (diff < -90.0) diff += 180.0;
  return diff;
}

/// 计算从 current 到 target 的最短路径目标值
inline double shortest_path_target(double target, double current) {
  return current + wrap_180(target - current);
}

}  // namespace detail

constexpr double kSteerGearRatio = 23.0 / 92.0;
constexpr double kWheelGearRatio = 1.0;
constexpr double kWheelRadiusMm = 25.0;
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
  double initial_angle_a{0.0};  // 手动标定时设置编码器零点 A，为 0 则自动捕获
  double initial_angle_b{0.0};  // 手动标定时设置编码器零点 B，为 0 则自动捕获
  int32_t rotation_port{-1};     // rotation sensor 端口，-1 表示不使用
  bool rotation_reversed{false}; // rotation sensor 是否反转
  double rotation_offset{0.0};   // 标定偏移：轮子物理指向 0° 时 rotation.angle() 的值
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
  /// 轮组极速 (m/s) = motor_max_rpm / 60 × 轮周长
  const double wheel_max_speed_mps_;
  /// 绝对编码器（可选，用于断电保持航向）
  vex::rotation* rotation_{nullptr};
  /// rotation sensor 标零偏移量
  double rotation_offset_{0.0};

public:
  WheelUnit(vex::motor&& motor_a, vex::motor&& motor_b,
             double init_angle_a, double init_angle_b,
             const basic::control::pid::Pid::Config& vpid_cfg,
             const basic::control::pid::Pid::Config& hpid_cfg,
             const basic::control::pid::Pid::Config& avpid_cfg,
             const first_order_adrc::Controller::Config& adrc_a_cfg,
             const first_order_adrc::Controller::Config& adrc_b_cfg,
             double motor_max_rpm,
             vex::rotation* rotation = nullptr,
             double rotation_offset = 0.0)
      : motor_a_(std::move(motor_a)),
        motor_b_(std::move(motor_b)),
        state_{0.0, 0.0, 0.0, init_angle_a, init_angle_b},
        velocity_pid(vpid_cfg),
        heading_pid(hpid_cfg),
        angular_velocity_pid(avpid_cfg),
        adrc_a_(adrc_a_cfg),
        adrc_b_(adrc_b_cfg),
        pct_to_mps_factor_(motor_max_rpm / 100.0 / 60.0 * kWheelCircumference),
        wheel_max_speed_mps_(motor_max_rpm / 60.0 * kWheelCircumference),
        rotation_(rotation),
        rotation_offset_(rotation_offset) {}

  double wheel_max_speed_mps() const { return wheel_max_speed_mps_; }
  double heading() const { return state_.heading; }

  /// 机械标零：将当前物理位置记录为航向 0°，重置所有控制器
  void calibrate_zero() {
    if (rotation_ != nullptr) {
      rotation_offset_ = rotation_->angle(vex::deg);
    } else {
      state_.initial_angle_a = motor_a_.position(vex::deg);
      state_.initial_angle_b = motor_b_.position(vex::deg);
    }
    velocity_pid.reset();
    heading_pid.reset();
    angular_velocity_pid.reset();
    update();
  }

  /// 打印当前编码器位置与航向（用于手动标定）
  void print_position() {
    if (rotation_ != nullptr) {
      printf("  rot_angle=%.1f deg  rot_offset=%.1f deg  heading=%.1f deg\n",
             rotation_->angle(vex::deg), rotation_offset_, state_.heading);
    } else {
      printf("  motor_a=%.1f deg  motor_b=%.1f deg  heading=%.1f deg\n",
             motor_a_.position(vex::deg), motor_b_.position(vex::deg), state_.heading);
    }
  }

  void update() {
    const double angle_a = motor_a_.position(vex::deg) - state_.initial_angle_a;
    const double angle_b = motor_b_.position(vex::deg) - state_.initial_angle_b;

    const double vel_a = motor_a_.velocity(vex::pct);
    const double vel_b = motor_b_.velocity(vex::pct);

    // 航向：优先使用绝对编码器（rotation sensor），否则用电机差速推算
    if (rotation_ != nullptr) {
      state_.heading = detail::wrap_180(rotation_->angle(vex::deg) - rotation_offset_);
    } else {
      state_.heading = detail::wrap_180((angle_a + angle_b) * 0.5 * kSteerGearRatio);
    }
    // steer/vel=差速器解算
    state_.steer_velocity = (vel_a + vel_b) * 0.5 * kSteerGearRatio;
    state_.velocity = (vel_a - vel_b) * 0.5 * kWheelGearRatio;
  }

  void control(double target_velocity_pct,
               double target_heading,
               vex::brakeType brake_type) {
    update();

    // 轮速目标 pct → m/s（运动学输出为 pct，在此处转为物理量）
    const double target_mps = target_velocity_pct / 100.0 * wheel_max_speed_mps_;
    // 反馈值 pct → m/s
    const double feedback_mps = state_.velocity * pct_to_mps_factor_;

    // 航向误差（模 180°：0°与180°等价），取短路径，映射到 [-90, 90]
    const double raw_error = detail::wrap_180(target_heading - state_.heading);
    const double heading_error = detail::wrap_90(raw_error);
    const bool flip = std::abs(raw_error) > 90.0;

    // 航向角位置环 — 以 heading_error 为 setpoint、0 为 feedback
    // PID 内部 error = heading_error - 0 = heading_error
    const auto heading_result = heading_pid.update(heading_error, 0.0);

    // 航向角速度环
    const auto steer_result = angular_velocity_pid.update(
        heading_result.ctrl, state_.steer_velocity);

    // 轮速环（翻转时速度取反，轮速反向等效于航向转180°）
    const double effective_target_mps = flip ? -target_mps : target_mps;
    const auto velocity_result = velocity_pid.update(effective_target_mps, feedback_mps);

    // 复位判定（复用 heading_error，避免二次 wrap_180）
    if (std::abs(heading_error) < 0.5 && std::abs(state_.steer_velocity) < 0.5) {
      heading_pid.reset();
      angular_velocity_pid.reset();
    }
    if (std::abs(effective_target_mps) < 0.01 && std::abs(feedback_mps) < 0.01) {
      velocity_pid.reset();
    }

    // 组合输出，超限时等比例缩放
    const double v = velocity_result.ctrl;
    const double s = steer_result.ctrl;
    const double a_speed =0.5 * (v + s);
    const double b_speed =0.5 * (-v + s);
    const double max_abs = std::max(std::abs(a_speed), std::abs(b_speed));
    const double scale = (max_abs > 100.0) ? (100.0 / max_abs) : 1.0;

    //basic::control::adrc_torque_control(motor_a_, a_speed * scale, adrc_a_);
    //basic::control::adrc_torque_control(motor_b_, b_speed * scale, adrc_b_);
    basic::control::velocitycontrol(motor_a_, a_speed * scale, vex::pct);
    basic::control::velocitycontrol(motor_b_, b_speed * scale, vex::pct);

    /*printf("W|v_in:%.0f v_tgt:%.2f v_fb:%.2f v_out:%.0f | h_tgt:%.0f h_fb:%.0f h_err:%.0f h_out:%.0f | s_out:%.0f a:%.0f b:%.0f\n",
           target_velocity_pct, target_mps, feedback_mps, v * scale,
           target_heading, state_.heading, heading_error, heading_result.ctrl,
           s * scale,
           a_speed * scale, b_speed * scale);*/
  }
};

namespace detail {
inline WheelUnit make_wheel_unit(const WheelUnitConfig& config) {
  vex::motor motor_a{config.motor_a.port, config.motor_a.gear_ratio, config.motor_a.reversed};
  vex::motor motor_b{config.motor_b.port, config.motor_b.gear_ratio, config.motor_b.reversed};
  // 手动标定：config 中 initial_angle 非零则使用配置值，否则自动捕获编码器位置
  const bool manual_calib = config.initial_angle_a != 0.0 || config.initial_angle_b != 0.0;
  double init_angle_a, init_angle_b;
  if (manual_calib) {
    init_angle_a = config.initial_angle_a;
    init_angle_b = config.initial_angle_b;
  } else {
    vex::this_thread::sleep_for(50);
    init_angle_a = motor_a.position(vex::deg);
    init_angle_b = motor_b.position(vex::deg);
  }

  // 构造绝对编码器（若已安装）
  vex::rotation* rotation = nullptr;
  if (config.rotation_port >= 0) {
    rotation = new vex::rotation(config.rotation_port, config.rotation_reversed);
  }

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
      rotation,
      config.rotation_offset,
  };
}
}  // namespace detail
}  // namespace basic::chassis::steering

#endif // BASIC_INCLUDE_WHEEL_UNIT_H
