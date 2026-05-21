#include "control/chassis.h"

#include "control/motor_control.h"

#include <algorithm>
#include <cmath>

namespace basic::hardware::robots {

namespace {

void apply_motor_power(vex::motor& motor, double speed, vex::brakeType type) {
  if (speed) {
    velocitycontrol(motor, speed, vex::pct);
  } else {
    stopcontrol(motor, type);
  }
}

double shape_input(double input) {
  const bool negative = input < 0;
  const double normalized = std::abs(input) * 0.01;
  const double shaped = normalized * normalized * (3 - 2 * normalized) * 100.0;
  return negative ? -shaped : shaped;
}

double dynamic_smooth(int now, int last, double rating) {
  if (std::abs(now) > kDeadZone) {
    const double ratio = 0.4 + 0.5 * rating;
    return now * ratio + last * (1 - ratio);
  }

  const double ratio = 0.7 + 0.2 * rating;
  return last * (1 - ratio);
}

}  // namespace

void chassis_update(RobotHardware& hardware, RobotState& state) {
  const ControllerInputState& input = state.controller;
  const SensorState& sensors = state.sensors;
  (void)sensors;

  const double axis3 = dynamic_smooth(input.axis3, input.last_axis3, input.rating[2]);
  const double axis1 = dynamic_smooth(input.axis1, input.last_axis1, input.rating[0]);
  double front_left = axis3 + axis1;
  double front_right = axis3 - axis1;
  double back_left = axis3 + axis1;
  double back_right = axis3 - axis1;

  const double maxpct = std::max({std::fabs(front_left), std::fabs(front_right), std::fabs(back_left), std::fabs(back_right)});
  if (maxpct > 100) {
    const double ratio = 100.0 / maxpct;
    front_left *= ratio;
    front_right *= ratio;
    back_left *= ratio;
    back_right *= ratio;
  }

  ChassisState& chassis = state.chassis;
  chassis.fl = shape_input(front_left);
  chassis.fr = shape_input(front_right);
  chassis.bl = shape_input(back_left);
  chassis.br = shape_input(back_right);

  apply_motor_power(hardware.motor_bl1, chassis.bl, chassis.stop_brake_type);
  apply_motor_power(hardware.motor_bl2, chassis.bl, chassis.stop_brake_type);
  apply_motor_power(hardware.motor_br1, chassis.br, chassis.stop_brake_type);
  apply_motor_power(hardware.motor_br2, chassis.br, chassis.stop_brake_type);
  apply_motor_power(hardware.motor_fl1, chassis.fl, chassis.stop_brake_type);
  apply_motor_power(hardware.motor_fl2, chassis.fl, chassis.stop_brake_type);
  apply_motor_power(hardware.motor_fr1, chassis.fr, chassis.stop_brake_type);
  apply_motor_power(hardware.motor_fr2, chassis.fr, chassis.stop_brake_type);

}

}  // namespace basic::hardware::robots
