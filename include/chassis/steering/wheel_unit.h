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

inline double clamp_value(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

}  // namespace detail

constexpr double kSteerGearRatio = 23.0 / 92.0;
constexpr double kHalfSteerGearRatio = kSteerGearRatio * 0.5;  // 预计算，避免每帧乘法
constexpr double kWheelGearRatio = 1.0;
constexpr double kHalfWheelGearRatio = kWheelGearRatio * 0.5;  // = 0.5
constexpr double kWheelRadiusMm = 25.0;
constexpr double kWheelCircumference = 2.0 * M_PI * kWheelRadiusMm * 1e-3; // m
constexpr double kMotorOutputLimitPct = 100.0;
constexpr double kSharedMotorBudgetPct = 2.0 * kMotorOutputLimitPct;
/// 轮速前馈增益：差速器正解 轮速 = (a-b)×0.5，故差速预算需 ×(1/0.5) 使「轮速 pct = 目标 pct」
constexpr double kWheelVelocityFeedforward = 1.0 / kHalfWheelGearRatio;  // = 2.0

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
  vex::triport::port* analog_port{nullptr};  // 三线模拟编码器端口（0-4095），nullptr 表示不使用
  double analog_full_scale_deg{360.0};       // 模拟满量程 0-4095 对应的角度行程
  bool analog_reversed{false};               // 读数方向反转（装反时改 true）
  double analog_zero_raw{0.0};               // 三线编码手动基准（raw），0 = 上电自动捕获
  double analog_deadband_raw{1.0};           // 模拟读数滞环死区（raw 域，±1 = ±0.088°）
  int debug_id{0};                           // 调试打印 ID（0=FR, 1=FL, 2=BR, 3=BL）

  /// 便捷构造：统一 PID 配置，ADRC 使用默认值
  static inline WheelUnitConfig simple(
      basic::device::MotorConfig motor_a,
      basic::device::MotorConfig motor_b,
      const basic::control::pid::Pid::Config& velo_pid,
      const basic::control::pid::Pid::Config& head_pid,
      const basic::control::pid::Pid::Config& av_pid,
      double max_rpm = 200) {
    return {motor_a, motor_b, velo_pid, head_pid, av_pid,
            {}, {}, max_rpm};
  }
};

struct WheelUnitState {
  double velocity{0.0};
  double heading{0.0};
  double steer_velocity{0.0};
  double analog_deg{0.0};   // 融合前模拟编码器角度（调试用）
  double motor_steer_deg{0.0};  // 电机差速转向角（调试用）
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
  /// 三线模拟绝对编码器（可选，断电保持航向）
  vex::analog_in* analog_{nullptr};
  /// 模拟满量程 0-4095 对应的角度行程
  double analog_full_scale_deg_{360.0};
  /// 读数方向反转
  bool analog_reversed_{false};
  /// 标零偏移（raw 域）：轮子物理指向 0° 时的模拟读数
  double analog_zero_raw_{0.0};
  /// 模拟读数滞环死区（raw 域）
  double analog_deadband_raw_{1.0};
  /// 上次接受的模拟读数（滞环基准）
  double last_accepted_raw_{0.0};
  /// 调试打印 ID
  int debug_id_{0};

public:
  WheelUnit(vex::motor&& motor_a, vex::motor&& motor_b,
             double init_angle_a, double init_angle_b,
             const basic::control::pid::Pid::Config& vpid_cfg,
             const basic::control::pid::Pid::Config& hpid_cfg,
             const basic::control::pid::Pid::Config& avpid_cfg,
             const first_order_adrc::Controller::Config& adrc_a_cfg,
             const first_order_adrc::Controller::Config& adrc_b_cfg,
             double motor_max_rpm,
             vex::analog_in* analog = nullptr,
             double analog_full_scale_deg = 360.0,
             bool analog_reversed = false,
             double analog_zero_raw = 0.0,
             double analog_deadband_raw = 1.0,
             int debug_id = 0)
      : motor_a_(std::move(motor_a)),
        motor_b_(std::move(motor_b)),
        state_{0.0, 0.0, 0.0, 0.0, 0.0, init_angle_a, init_angle_b},
        velocity_pid(vpid_cfg),
        heading_pid(hpid_cfg),
        angular_velocity_pid(avpid_cfg),
        adrc_a_(adrc_a_cfg),
        adrc_b_(adrc_b_cfg),
        pct_to_mps_factor_(motor_max_rpm / 100.0 / 60.0 * kWheelCircumference),
        wheel_max_speed_mps_(motor_max_rpm / 60.0 * kWheelCircumference),
        analog_(analog),
        analog_full_scale_deg_(analog_full_scale_deg),
        analog_reversed_(analog_reversed),
        analog_zero_raw_(analog_zero_raw),
        last_accepted_raw_(analog_zero_raw),
        analog_deadband_raw_(analog_deadband_raw),
        debug_id_(debug_id) {}

  double wheel_max_speed_mps() const { return wheel_max_speed_mps_; }
  double heading() const { return state_.heading; }

  /// 读取模拟编码器 raw（统一做反向镜像，保证所有使用点域一致）
  double read_analog_raw() const {
    if (analog_ == nullptr) {
      return 0.0;
    }
    double raw = analog_->value(vex::analogUnits::range12bit);
    if (analog_reversed_) {
      raw = 4095.0 - raw;
    }
    return raw;
  }

  /// 机械标零：将当前物理位置记录为航向 0°，重置所有控制器
  void calibrate_zero() {
    if (analog_ != nullptr) {
      analog_zero_raw_ = read_analog_raw();   // 与 update() 同域（已镜像）
      last_accepted_raw_ = analog_zero_raw_;  // 滞环基准同步到新零位
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
    if (analog_ != nullptr) {
      printf("  analog_raw=%.0f  zero=%.0f  heading=%.1f deg\n",
             read_analog_raw(), analog_zero_raw_, state_.heading);
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

    // 转向角（电机编码器差速，仅用于调试对比）
    const double motor_steer_deg = (angle_a + angle_b) * kHalfSteerGearRatio;
    state_.motor_steer_deg = motor_steer_deg;  // 记录（调试）

    // 航向：直接使用三线模拟绝对编码器（无混合滤波）
    if (analog_ != nullptr) {
      const double raw = read_analog_raw();
      // 滞环死区：与上次接受值差在 ±deadband 内保持旧值，抑制 ADC 抖动
      if (std::fabs(raw - last_accepted_raw_) > analog_deadband_raw_) {
        last_accepted_raw_ = raw;
      }
      state_.heading = detail::wrap_180(
          (last_accepted_raw_ - analog_zero_raw_) / 4095.0 * analog_full_scale_deg_);
      state_.analog_deg = state_.heading;  // 与航向一致（调试）
    } else {
      state_.heading = detail::wrap_180(motor_steer_deg);
    }
    // steer/vel=差速器解算
    state_.steer_velocity = (vel_a + vel_b) * kHalfSteerGearRatio;
    state_.velocity = (vel_a - vel_b) * kHalfWheelGearRatio;
  }

  void control(double target_velocity_pct,
               double target_heading,
               vex::brakeType brake_type) {
    update();

    // 调试打印：仅第一个轮组（FR）输出，节流每 10 拍 1 次
    if (debug_id_ == 0) {
      static int print_tick = 0;
      if (++print_tick >= 10) {
        print_tick = 0;
        printf("W[%d]|raw:%.0f analog:%.2f motor:%.2f target:%.2f fused:%.2f\n",
               debug_id_, last_accepted_raw_, state_.analog_deg,
               state_.motor_steer_deg, target_heading, state_.heading);
      }
    }

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

    // 轮速前馈：目标 pct 直接映射到差速预算域（×2 抵消差速器 0.5 因子）。
    // 前馈承担主输出，PID 只做扰动/摩擦修正，PID 参数无需为满速服务。
    const double effective_target_pct = flip ? -target_velocity_pct : target_velocity_pct;
    const double v_ff = kWheelVelocityFeedforward * effective_target_pct;

    // 复位判定（复用 heading_error，避免二次 wrap_180）
    if (std::abs(heading_error) < 0.5 && std::abs(state_.steer_velocity) < 0.5) {
      heading_pid.reset();
      angular_velocity_pid.reset();
    }
    if (std::abs(effective_target_mps) < 0.01 && std::abs(feedback_mps) < 0.01) {
      velocity_pid.reset();
    }

    // 组合输出，超限时等比例缩放
    const double s = detail::clamp_value(
        steer_result.ctrl, -kSharedMotorBudgetPct, kSharedMotorBudgetPct);
    const double drive_budget = std::max(0.0, kSharedMotorBudgetPct - std::abs(s));
    const double v = detail::clamp_value(
        v_ff + velocity_result.ctrl, -drive_budget, drive_budget);
    const double a_speed = 0.5 * (v + s);
    const double b_speed = 0.5 * (-v + s);

    //basic::control::adrc_torque_control(motor_a_, a_speed, adrc_a_);
    //basic::control::adrc_torque_control(motor_b_, b_speed, adrc_b_);
    basic::control::velocitycontrol(
        motor_a_,
        detail::clamp_value(
            a_speed, -kMotorOutputLimitPct, kMotorOutputLimitPct),
        vex::pct);
    basic::control::velocitycontrol(
        motor_b_,
        detail::clamp_value(
            b_speed, -kMotorOutputLimitPct, kMotorOutputLimitPct),
        vex::pct);

    /*printf("W|v_in:%.0f v_tgt:%.2f v_fb:%.2f v_out:%.0f | h_tgt:%.0f h_fb:%.0f h_err:%.0f h_out:%.0f | s_out:%.0f a:%.0f b:%.0f\n",
           target_velocity_pct, target_mps, feedback_mps, v,
           target_heading, state_.heading, heading_error, heading_result.ctrl,
           s,
           a_speed, b_speed);*/
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

  // 构造三线模拟绝对编码器（若已安装）
  vex::analog_in* analog = nullptr;
  if (config.analog_port != nullptr) {
    analog = new vex::analog_in(*config.analog_port);
  }

  // 三线编码基准：手动值（非 0）优先，否则上电自动捕获当前读数为 0°
  double analog_zero_raw = config.analog_zero_raw;
  if (analog != nullptr && analog_zero_raw == 0.0) {
    vex::this_thread::sleep_for(50);  // 等待模拟通道稳定
    analog_zero_raw = analog->value(vex::analogUnits::range12bit);
    if (config.analog_reversed) {
      analog_zero_raw = 4095.0 - analog_zero_raw;
    }
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
      analog,
      config.analog_full_scale_deg,
      config.analog_reversed,
      analog_zero_raw,
      config.analog_deadband_raw,
      config.debug_id,
  };
}
}  // namespace detail
}  // namespace basic::chassis::steering

#endif // BASIC_INCLUDE_WHEEL_UNIT_H
