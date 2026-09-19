#include "input/controller.h"

#include <cmath>

namespace basic::input {

namespace {

using basic::hardware::shared::ControllerInputState;

/// 按键帧保护状态：level = 已确认电平，count = 连续反向帧计数
struct ButtonFilter {
  bool level{false};
  int count{0};
};

/// 12 个按键的帧保护状态（顺序与下方 controller_update 中一致）
ButtonFilter button_filters[12];

/// 帧保护：raw 电平需连续 frames 帧与当前 level 不同才翻转（按下、松开对称）
/// frames <= 1 时直接透传
/// previous_confirmed：上一帧对外输出的已确认电平。若它被外部清零
/// （如 stop_all_outputs 里 state.controller = {}），则同步复位本滤波器，
/// 避免"复位后沿用旧的按下状态"导致误触发/绕过保护。
bool debounce_button(
    int index,
    bool raw_pressed,
    int frames,
    bool previous_confirmed) {
  ButtonFilter& filter = button_filters[index];

  if (!previous_confirmed && filter.level) {
    filter.level = false;  // 外部复位同步
    filter.count = 0;
  }

  if (frames <= 1 || raw_pressed == filter.level) {
    filter.count = 0;
    if (frames <= 1) {
      filter.level = raw_pressed;
    }
    return filter.level;
  }

  if (++filter.count >= frames) {
    filter.level = raw_pressed;
    filter.count = 0;
  }
  return filter.level;
}

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

void controller_update(
    vex::brain& brain,
    vex::controller& controller,
    basic::hardware::shared::ControllerInputState& input,
    int button_debounce_frames,
    int axis_snap_pct) {

  input.last_axis1 = input.axis1;
  input.last_axis2 = input.axis2;
  input.last_axis3 = input.axis3;
  input.last_axis4 = input.axis4;

  // 上一帧（已确认）按键电平，用于 press_* 按下沿判定
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

  input.time_ms = brain.timer(vex::timeUnits::msec);

  input.axis1 = controller.Axis1.position(vex::percentUnits::pct);
  input.axis2 = controller.Axis2.position(vex::percentUnits::pct);
  input.axis3 = controller.Axis3.position(vex::percentUnits::pct);
  input.axis4 = controller.Axis4.position(vex::percentUnits::pct);

  // 零位吸附：松手时手柄 ADC 仍有 ±1 的量化抖动，|值| ≤ snap 时按 0 输出
  // （0 = 关闭；底盘死区能挡住它，但航向环的摇杆积分不希望把抖动积进目标 yaw）
  if (axis_snap_pct > 0) {
    if (std::abs(input.axis1) <= axis_snap_pct) input.axis1 = 0;
    if (std::abs(input.axis2) <= axis_snap_pct) input.axis2 = 0;
    if (std::abs(input.axis3) <= axis_snap_pct) input.axis3 = 0;
    if (std::abs(input.axis4) <= axis_snap_pct) input.axis4 = 0;
  }

  // 按键：原始读数 → 帧保护（按下/松开均需连续 N 帧确认）
  // 传入上一帧已确认电平（input.last_*，此刻尚未被覆盖）用于外部复位同步
  input.l1 = debounce_button(0, controller.ButtonL1.pressing(), button_debounce_frames, input.last_l1);
  input.l2 = debounce_button(1, controller.ButtonL2.pressing(), button_debounce_frames, input.last_l2);
  input.r1 = debounce_button(2, controller.ButtonR1.pressing(), button_debounce_frames, input.last_r1);
  input.r2 = debounce_button(3, controller.ButtonR2.pressing(), button_debounce_frames, input.last_r2);
  input.up = debounce_button(4, controller.ButtonUp.pressing(), button_debounce_frames, input.last_up);
  input.down = debounce_button(5, controller.ButtonDown.pressing(), button_debounce_frames, input.last_down);
  input.left = debounce_button(6, controller.ButtonLeft.pressing(), button_debounce_frames, input.last_left);
  input.right = debounce_button(7, controller.ButtonRight.pressing(), button_debounce_frames, input.last_right);
  input.x = debounce_button(8, controller.ButtonX.pressing(), button_debounce_frames, input.last_x);
  input.y = debounce_button(9, controller.ButtonY.pressing(), button_debounce_frames, input.last_y);
  input.a = debounce_button(10, controller.ButtonA.pressing(), button_debounce_frames, input.last_a);
  input.b = debounce_button(11, controller.ButtonB.pressing(), button_debounce_frames, input.last_b);

  clear_press_events(input);
  calculate_button_rating(input);
  update_press_events(input);
}

}  // namespace basic::input
