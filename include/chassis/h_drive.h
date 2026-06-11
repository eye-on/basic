#ifndef BASIC_INCLUDE_H_DRIVE_H_
#define BASIC_INCLUDE_H_DRIVE_H_

#include "chassis/arcade_drive.h"

namespace basic::chassis {

struct HDriveCommand {
  int forward_input_pct{0};
  int last_forward_input_pct{0};
  double forward_rating{0.0};
  int strafe_input_pct{0};
  int last_strafe_input_pct{0};
  double strafe_rating{0.0};
  int turn_input_pct{0};
  int last_turn_input_pct{0};
  double turn_rating{0.0};
  vex::brakeType stop_brake_type{vex::coast};
};

struct HDriveState {
  double left_pct{0.0};
  double right_pct{0.0};
  double center_pct{0.0};
  vex::brakeType stop_brake_type{vex::coast};
};

template <std::size_t LeftCount, std::size_t RightCount>
struct HDriveConfig {
  std::array<basic::device::MotorConfig, LeftCount> left_motors;
  std::array<basic::device::MotorConfig, RightCount> right_motors;
  basic::device::MotorConfig center_motor;
  int deadzone{10};
};

template <std::size_t LeftCount, std::size_t RightCount>
class HDrive {
 public:
  explicit HDrive(const HDriveConfig<LeftCount, RightCount>& config)
      : left_motors_(detail::make_motor_array(config.left_motors)),
        right_motors_(detail::make_motor_array(config.right_motors)),
        center_motor_(
            config.center_motor.port,
            config.center_motor.gear_ratio,
            config.center_motor.reversed),
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

  vex::motor& center_motor() {
    return center_motor_;
  }

  const vex::motor& center_motor() const {
    return center_motor_;
  }

  int deadzone() const {
    return deadzone_;
  }

  HDriveState& state() {
    return state_;
  }

  const HDriveState& state() const {
    return state_;
  }

 private:
  std::array<vex::motor, LeftCount> left_motors_;
  std::array<vex::motor, RightCount> right_motors_;
  vex::motor center_motor_;
  int deadzone_{10};
  HDriveState state_;
};

template <std::size_t LeftCount, std::size_t RightCount>
HDrive<LeftCount, RightCount> h_drive_init(
    const HDriveConfig<LeftCount, RightCount>& config) {
  return HDrive<LeftCount, RightCount>(config);
}

inline HDriveCommand h_drive_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input,
    ControllerAxis forward_axis,
    ControllerAxis strafe_axis,
    ControllerAxis turn_axis,
    vex::brakeType stop_brake_type = vex::coast) {
  return {
      controller_axis_value(input, forward_axis),
      controller_axis_previous_value(input, forward_axis),
      controller_axis_rating(input, forward_axis),
      controller_axis_value(input, strafe_axis),
      controller_axis_previous_value(input, strafe_axis),
      controller_axis_rating(input, strafe_axis),
      controller_axis_value(input, turn_axis),
      controller_axis_previous_value(input, turn_axis),
      controller_axis_rating(input, turn_axis),
      stop_brake_type,
  };
}

template <std::size_t LeftCount, std::size_t RightCount>
void h_drive_set_output(
    HDrive<LeftCount, RightCount>& chassis,
    double left_pct,
    double right_pct,
    double center_pct,
    vex::brakeType brake_type) {
  chassis.state().left_pct = left_pct;
  chassis.state().right_pct = right_pct;
  chassis.state().center_pct = center_pct;
  chassis.state().stop_brake_type = brake_type;

  detail::apply_group_output(chassis.left_motors(), left_pct, brake_type);
  detail::apply_group_output(chassis.right_motors(), right_pct, brake_type);
  detail::set_motor_output(chassis.center_motor(), center_pct, brake_type);
}

template <std::size_t LeftCount, std::size_t RightCount>
void h_drive_update(
    HDrive<LeftCount, RightCount>& chassis,
    const HDriveCommand& command) {
  const double forward = detail::dynamic_smooth(
      command.forward_input_pct,
      command.last_forward_input_pct,
      command.forward_rating,
      chassis.deadzone());
  const double strafe = detail::dynamic_smooth(
      command.strafe_input_pct,
      command.last_strafe_input_pct,
      command.strafe_rating,
      chassis.deadzone());
  const double turn = detail::dynamic_smooth(
      command.turn_input_pct,
      command.last_turn_input_pct,
      command.turn_rating,
      chassis.deadzone());

  double left_pct = forward + turn;
  double right_pct = forward - turn;
  double center_pct = strafe;

  const double max_pct = std::max(
      {std::fabs(left_pct), std::fabs(right_pct), std::fabs(center_pct)});
  if (max_pct > 100.0) {
    const double scale = 100.0 / max_pct;
    left_pct *= scale;
    right_pct *= scale;
    center_pct *= scale;
  }

  h_drive_set_output(
      chassis,
      detail::shape_input(left_pct),
      detail::shape_input(right_pct),
      detail::shape_input(center_pct),
      command.stop_brake_type);
}

template <std::size_t LeftCount, std::size_t RightCount>
void h_drive_stop(
    HDrive<LeftCount, RightCount>& chassis,
    vex::brakeType brake_type = vex::coast) {
  h_drive_set_output(chassis, 0.0, 0.0, 0.0, brake_type);
}

template <std::size_t LeftCount, std::size_t RightCount>
HDriveState& h_drive_state(HDrive<LeftCount, RightCount>& chassis) {
  return chassis.state();
}

template <std::size_t LeftCount, std::size_t RightCount>
const HDriveState& h_drive_state(const HDrive<LeftCount, RightCount>& chassis) {
  return chassis.state();
}

}  // namespace basic::chassis

#endif  // BASIC_INCLUDE_H_DRIVE_H_
