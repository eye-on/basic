#include "control/second_robot/chassis.h"

#include "control/motor_control.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace basic::control::second_robot {

namespace {

using basic::control::stopcontrol;
using basic::control::velocitycontrol;
using basic::hardware::second_robot::RobotHardware;
using basic::hardware::second_robot::RobotState;

double shape_input(double input) {
  const bool negative = input < 0.0;
  const double normalized = std::abs(input) * 0.01;
  const double shaped = normalized * normalized * (3.0 - 2.0 * normalized) * 100.0;
  return negative ? -shaped : shaped;
}

double dynamic_smooth(int now, int last, double rating) {
  if (std::abs(now) > basic::hardware::second_robot::kDeadZone) {
    const double ratio = 0.4 + 0.5 * rating;
    return now * ratio + last * (1.0 - ratio);
  }

  const double ratio = 0.7 + 0.2 * rating;
  return last * (1.0 - ratio);
}

void apply_side_power(const std::array<vex::motor*, 3>& motors, double speed, vex::brakeType brake_type) {
  for (vex::motor* motor : motors) {
    if (speed != 0.0) {
      velocitycontrol(*motor, speed, vex::pct);
    } else {
      stopcontrol(*motor, brake_type);
    }
  }
}

}  // namespace

void chassis_update(RobotHardware& hardware, RobotState& state) {
  const basic::hardware::shared::ControllerInputState& input = state.controller;
  const double forward = dynamic_smooth(input.axis3, input.last_axis3, input.rating[2]);
  const double turn = dynamic_smooth(input.axis1, input.last_axis1, input.rating[0]);

  double left = forward + turn;
  double right = forward - turn;
  const double maxpct = std::max(std::fabs(left), std::fabs(right));
  if (maxpct > 100.0) {
    const double ratio = 100.0 / maxpct;
    left *= ratio;
    right *= ratio;
  }

  state.chassis.left_pct = shape_input(left);
  state.chassis.right_pct = shape_input(right);

  apply_side_power(
      {{&hardware.left_front_motor, &hardware.left_middle_motor, &hardware.left_back_motor}},
      state.chassis.left_pct,
      state.chassis.stop_brake_type);
  apply_side_power(
      {{&hardware.right_front_motor, &hardware.right_middle_motor, &hardware.right_back_motor}},
      state.chassis.right_pct,
      state.chassis.stop_brake_type);
}

}  // namespace basic::control::second_robot
