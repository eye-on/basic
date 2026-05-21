#include "input/controller.h"

#include <cmath>

namespace basic::hardware::robots {

namespace {

void calculate_button_rating(ControllerInputState& state) {
  state.rating[0] = std::abs(state.axis1 - state.last_axis1) * 0.005;
  state.rating[1] = std::abs(state.axis2 - state.last_axis2) * 0.005;
  state.rating[2] = std::abs(state.axis3 - state.last_axis3) * 0.005;
  state.rating[3] = std::abs(state.axis4 - state.last_axis4) * 0.005;
}

void clear_press_events(ControllerInputState& state) {
  state.press_x = false;
  state.press_y = false;
  state.press_a = false;
  state.press_b = false;
  state.press_up = false;
  state.press_down = false;
  state.press_left = false;
  state.press_right = false;
  state.press_l1 = false;
  state.press_l2 = false;
  state.press_r1 = false;
  state.press_r2 = false;
}

void update_press_events(ControllerInputState& state) {
  state.press_x = state.x && !state.last_x;
  state.press_a = state.a && !state.last_a;
  state.press_b = state.b && !state.last_b;
  state.press_y = state.y && !state.last_y;
  state.press_up = state.up && !state.last_up;
  state.press_down = state.down && !state.last_down;
  state.press_right = state.right && !state.last_right;
  state.press_left = state.left && !state.last_left;
  state.press_l1 = state.l1 && !state.last_l1;
  state.press_l2 = state.l2 && !state.last_l2;
  state.press_r1 = state.r1 && !state.last_r1;
  state.press_r2 = state.r2 && !state.last_r2;
}

}  // namespace

void controller_update(RobotHardware& hardware, RobotState& state) {
  ControllerInputState& input = state.controller;

  input.last_axis1 = input.axis1;
  input.last_axis2 = input.axis2;
  input.last_axis3 = input.axis3;
  input.last_axis4 = input.axis4;

  input.last_l1 = input.l1;
  input.last_l2 = input.l2;
  input.last_r1 = input.r1;
  input.last_r2 = input.r2;
  input.last_x = input.x;
  input.last_y = input.y;
  input.last_a = input.a;
  input.last_b = input.b;
  input.last_left = input.left;
  input.last_right = input.right;
  input.last_up = input.up;
  input.last_down = input.down;

  input.time_ms = hardware.brain.timer(vex::timeUnits::msec);

  input.axis1 = hardware.controller.Axis1.position(vex::percentUnits::pct);
  input.axis2 = hardware.controller.Axis2.position(vex::percentUnits::pct);
  input.axis3 = hardware.controller.Axis3.position(vex::percentUnits::pct);
  input.axis4 = hardware.controller.Axis4.position(vex::percentUnits::pct);

  input.l1 = hardware.controller.ButtonL1.pressing();
  input.l2 = hardware.controller.ButtonL2.pressing();
  input.r1 = hardware.controller.ButtonR1.pressing();
  input.r2 = hardware.controller.ButtonR2.pressing();
  input.x = hardware.controller.ButtonX.pressing();
  input.y = hardware.controller.ButtonY.pressing();
  input.a = hardware.controller.ButtonA.pressing();
  input.b = hardware.controller.ButtonB.pressing();
  input.left = hardware.controller.ButtonLeft.pressing();
  input.right = hardware.controller.ButtonRight.pressing();
  input.up = hardware.controller.ButtonUp.pressing();
  input.down = hardware.controller.ButtonDown.pressing();

  clear_press_events(input);
  calculate_button_rating(input);
  update_press_events(input);
}

}  // namespace basic::hardware::robots
