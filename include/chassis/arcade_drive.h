#ifndef BASIC_INCLUDE_ARCADE_DRIVE_H_
#define BASIC_INCLUDE_ARCADE_DRIVE_H_

#include "control/motor_control.h"
#include "device_config.h"
#include "hardware/shared/state_types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>

namespace basic::chassis {

enum class ControllerAxis {
  kAxis1,
  kAxis2,
  kAxis3,
  kAxis4,
};

struct ArcadeDriveCommand {
  int forward_input_pct{0};
  int last_forward_input_pct{0};
  double forward_rating{0.0};
  int turn_input_pct{0};
  int last_turn_input_pct{0};
  double turn_rating{0.0};
  vex::brakeType stop_brake_type{vex::coast};
};

struct ArcadeDriveState {
  double left_pct{0.0};
  double right_pct{0.0};
  vex::brakeType stop_brake_type{vex::coast};
};

template <std::size_t LeftCount, std::size_t RightCount>
struct ArcadeDriveConfig {
  std::array<basic::device::MotorConfig, LeftCount> left_motors;
  std::array<basic::device::MotorConfig, RightCount> right_motors;
  int deadzone{10};
};

namespace detail {

template <std::size_t Count, std::size_t... Indices>
std::array<vex::motor, Count> make_motor_array_impl(
    const std::array<basic::device::MotorConfig, Count>& configs,
    std::index_sequence<Indices...>) {
  return {{
      vex::motor{configs[Indices].port, configs[Indices].gear_ratio, configs[Indices].reversed}...,
  }};
}

template <std::size_t Count>
std::array<vex::motor, Count> make_motor_array(
    const std::array<basic::device::MotorConfig, Count>& configs) {
  return make_motor_array_impl(configs, std::make_index_sequence<Count>{});
}

inline double shape_input(double input_pct) {
  const bool negative = input_pct < 0.0;
  const double normalized = std::abs(input_pct) * 0.01;
  const double shaped = normalized * normalized * (3.0 - 2.0 * normalized) * 100.0;
  return negative ? -shaped : shaped;
}

inline double dynamic_smooth(int current, int previous, double rating, int deadzone) {
  if (std::abs(current) > deadzone) {
    const double ratio = 0.4 + 0.5 * rating;
    return current * ratio + previous * (1.0 - ratio);
  }

  const double ratio = 0.7 + 0.2 * rating;
  return previous * (1.0 - ratio);
}

/// 力控（电压直驱）参数
/// pct → 电压因子：±100 pct = ±12000 mV（与 PROS 舵轮版一致）
inline constexpr double kPctToVoltageMv = 120.0;
/// 静摩擦补偿：指令非零但幅值过小时提升到最小可动电压（pct 域）
/// 消除"给了一点电压却不动"的死区；设为 0 可关闭
inline constexpr double kFrictionKickPct = 2.0;
/// 输出死区：小于此值视为停止（避免持续微小电压造成发热/嗡鸣）
/// 1.0 pct ≈ 120 mV
inline constexpr double kOutputDeadbandPct = 1.0;

/// 输出限幅：±100 pct
inline double clamp_pct(double pct) {
  if (pct > 100.0) {
    return 100.0;
  }
  if (pct < -100.0) {
    return -100.0;
  }
  return pct;
}

/// 组内平均实测转速（rpm）：软件速度环反馈 / 调试打印共用
template <std::size_t Count>
double group_rpm(std::array<vex::motor, Count>& motors) {
  double sum = 0.0;
  for (vex::motor& motor : motors) {
    sum += motor.velocity(vex::rpm);
  }
  return (Count > 0) ? sum / static_cast<double>(Count) : 0.0;
}

/// 组内首个电机的轴角度（度）：用于观察"每圈固定位置"的相位
template <std::size_t Count>
double group_position_deg(std::array<vex::motor, Count>& motors) {
  if (Count == 0) {
    return 0.0;
  }
  return motors[0].position(vex::deg);
}

/// 单电机下发（力控）：方向由符号决定，幅值 pct → 电压 mV
/// 扭矩 ∝ 电压，不经过电机固件速度环
inline void set_motor_output(vex::motor& motor, double pct, vex::brakeType brake_type) {
  if (pct > 100.0) pct = 100.0;
  if (pct < -100.0) pct = -100.0;

  const double mag = std::abs(pct);
  if (mag < kOutputDeadbandPct) {
    basic::control::stopcontrol(motor, brake_type);
    return;
  }

  double out_pct = pct;
  if (kFrictionKickPct > 0.0 && mag < kFrictionKickPct) {
    out_pct = (pct > 0.0) ? kFrictionKickPct : -kFrictionKickPct;
  }
  basic::control::voltagecontrol(motor, out_pct * kPctToVoltageMv);
}

template <std::size_t Count>
void apply_group_output(
    std::array<vex::motor, Count>& motors,
    double pct,
    vex::brakeType brake_type) {
  for (vex::motor& motor : motors) {
    set_motor_output(motor, pct, brake_type);
  }
}

/// 单电机下发（固件速度环）：幅值 pct 直接作为 VEX 固件速度环的目标速度
/// （pct 域，100 pct = 电机最高转速，与框架内 pct 语义一致）
/// 固件内部以远高于控制循环的速率闭环且带速度前馈，
/// 抗负载突变（如每圈固定位置的摩擦峰）远优于 100Hz 的软件环
inline void set_motor_velocity_output(
    vex::motor& motor,
    double pct,
    vex::brakeType brake_type) {
  if (pct > 100.0) pct = 100.0;
  if (pct < -100.0) pct = -100.0;

  if (std::abs(pct) < kOutputDeadbandPct) {
    basic::control::stopcontrol(motor, brake_type);
    return;
  }

  basic::control::velocitycontrol(motor, pct, vex::pct);
}

template <std::size_t Count>
void apply_group_velocity_output(
    std::array<vex::motor, Count>& motors,
    double pct,
    vex::brakeType brake_type) {
  for (vex::motor& motor : motors) {
    set_motor_velocity_output(motor, pct, brake_type);
  }
}

}  // namespace detail

template <std::size_t LeftCount, std::size_t RightCount>
class ArcadeDrive {
 public:
  explicit ArcadeDrive(const ArcadeDriveConfig<LeftCount, RightCount>& config)
      : left_motors_(detail::make_motor_array(config.left_motors)),
        right_motors_(detail::make_motor_array(config.right_motors)),
        deadzone_(config.deadzone) {}

  std::array<vex::motor, LeftCount>& left_motors() {
    return left_motors_;
  }

  const std::array<vex::motor, LeftCount>& left_motors() const {
    return left_motors_;
  }

  std::array<vex::motor, RightCount>& right_motors() {
    return right_motors_;
  }

  const std::array<vex::motor, RightCount>& right_motors() const {
    return right_motors_;
  }

  int deadzone() const {
    return deadzone_;
  }

  ArcadeDriveState& state() {
    return state_;
  }

  const ArcadeDriveState& state() const {
    return state_;
  }

 private:
  std::array<vex::motor, LeftCount> left_motors_;
  std::array<vex::motor, RightCount> right_motors_;
  int deadzone_{10};
  ArcadeDriveState state_;
};

template <std::size_t LeftCount, std::size_t RightCount>
ArcadeDrive<LeftCount, RightCount> arcade_init(
    const ArcadeDriveConfig<LeftCount, RightCount>& config) {
  return ArcadeDrive<LeftCount, RightCount>(config);
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

inline int controller_axis_previous_value(
    const basic::hardware::shared::ControllerInputState& input,
    ControllerAxis axis) {
  switch (axis) {
    case ControllerAxis::kAxis1:
      return input.last_axis1;
    case ControllerAxis::kAxis2:
      return input.last_axis2;
    case ControllerAxis::kAxis3:
      return input.last_axis3;
    case ControllerAxis::kAxis4:
    default:
      return input.last_axis4;
  }
}

inline double controller_axis_rating(
    const basic::hardware::shared::ControllerInputState& input,
    ControllerAxis axis) {
  switch (axis) {
    case ControllerAxis::kAxis1:
      return input.rating[0];
    case ControllerAxis::kAxis2:
      return input.rating[1];
    case ControllerAxis::kAxis3:
      return input.rating[2];
    case ControllerAxis::kAxis4:
    default:
      return input.rating[3];
  }
}

inline ArcadeDriveCommand arcade_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input,
    ControllerAxis forward_axis,
    ControllerAxis turn_axis,
    vex::brakeType stop_brake_type = vex::coast) {
  return {
      controller_axis_value(input, forward_axis),
      controller_axis_previous_value(input, forward_axis),
      controller_axis_rating(input, forward_axis),
      controller_axis_value(input, turn_axis),
      controller_axis_previous_value(input, turn_axis),
      controller_axis_rating(input, turn_axis),
      stop_brake_type,
  };
}

template <std::size_t LeftCount, std::size_t RightCount>
void arcade_set_output(
    ArcadeDrive<LeftCount, RightCount>& chassis,
    double left_pct,
    double right_pct,
    vex::brakeType brake_type) {
  chassis.state().left_pct = left_pct;
  chassis.state().right_pct = right_pct;
  chassis.state().stop_brake_type = brake_type;
  detail::apply_group_output(chassis.left_motors(), left_pct, brake_type);
  detail::apply_group_output(chassis.right_motors(), right_pct, brake_type);
}

template <std::size_t LeftCount, std::size_t RightCount>
void arcade_set_output(
    ArcadeDrive<LeftCount, RightCount>& chassis,
    double left_pct,
    double right_pct) {
  arcade_set_output(chassis, left_pct, right_pct, chassis.state().stop_brake_type);
}

template <std::size_t LeftCount, std::size_t RightCount>
void arcade_update(
    ArcadeDrive<LeftCount, RightCount>& chassis,
    const ArcadeDriveCommand& command) {
  const double forward = detail::dynamic_smooth(
      command.forward_input_pct,
      command.last_forward_input_pct,
      command.forward_rating,
      chassis.deadzone());
  const double turn = detail::dynamic_smooth(
      command.turn_input_pct,
      command.last_turn_input_pct,
      command.turn_rating,
      chassis.deadzone());

  double left_pct = forward + turn;
  double right_pct = forward - turn;
  const double max_pct = std::max(std::fabs(left_pct), std::fabs(right_pct));
  if (max_pct > 100.0) {
    const double scale = 100.0 / max_pct;
    left_pct *= scale;
    right_pct *= scale;
  }

  arcade_set_output(
      chassis,
      detail::shape_input(left_pct),
      detail::shape_input(right_pct),
      command.stop_brake_type);
}

template <std::size_t LeftCount, std::size_t RightCount>
void arcade_stop(
    ArcadeDrive<LeftCount, RightCount>& chassis,
    vex::brakeType brake_type = vex::coast) {
  arcade_set_output(chassis, 0.0, 0.0, brake_type);
}

template <std::size_t LeftCount, std::size_t RightCount>
ArcadeDriveState& arcade_state(ArcadeDrive<LeftCount, RightCount>& chassis) {
  return chassis.state();
}

template <std::size_t LeftCount, std::size_t RightCount>
const ArcadeDriveState& arcade_state(const ArcadeDrive<LeftCount, RightCount>& chassis) {
  return chassis.state();
}

}  // namespace basic::chassis

#endif
