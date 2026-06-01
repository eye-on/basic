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

inline void set_motor_output(vex::motor& motor, double pct, vex::brakeType brake_type) {
  if (pct != 0.0) {
    basic::control::velocitycontrol(motor, pct, vex::pct);
  } else {
    basic::control::stopcontrol(motor, brake_type);
  }
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
