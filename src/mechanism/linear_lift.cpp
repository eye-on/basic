#include "mechanism/linear_lift.h"

#include <algorithm>
#include <cmath>

#include "control/motor_control.h"

namespace basic::mechanism {

namespace {

using basic::control::get_done;
using basic::control::get_position;
using basic::control::stopcontrol;
using basic::control::velocitycontrol;

vex::motor make_motor(const basic::device::MotorConfig& config) {
  return vex::motor{config.port, config.gear_ratio, config.reversed};
}

void refresh_state(LinearLift& mechanism) {
  auto& s = mechanism.state();
  const auto& c = mechanism.config();

  s.motor1_position = get_position(mechanism.lift_motor1(), c.position_units);
  s.motor2_position = get_position(mechanism.lift_motor2(), c.position_units);

  s.at_target = get_done(mechanism.lift_motor1()) && get_done(mechanism.lift_motor2());

  s.synced = std::fabs(s.motor1_position - s.motor2_position) <= c.sync_max_deviation;
}

}  // namespace

LinearLift::LinearLift(const LinearLiftConfig& config)
    : config_(config),
      lift_motor1_(make_motor(config.lift_motor1.motor)),
      lift_motor2_(make_motor(config.lift_motor2.motor)) {}

vex::motor& LinearLift::lift_motor1() { return lift_motor1_; }
vex::motor& LinearLift::lift_motor2() { return lift_motor2_; }

const vex::motor& LinearLift::lift_motor1() const { return lift_motor1_; }
const vex::motor& LinearLift::lift_motor2() const { return lift_motor2_; }

LinearLiftConfig& LinearLift::config() { return config_; }
const LinearLiftConfig& LinearLift::config() const { return config_; }

LinearLiftState& LinearLift::state() { return state_; }
const LinearLiftState& LinearLift::state() const { return state_; }

LinearLift linear_lift_init(const LinearLiftConfig& config) {
  return LinearLift(config);
}

LinearLiftCommand linear_lift_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input) {
  LinearLiftCommand command;
  command.toggle_up = input.press_up;
  command.toggle_down = input.press_down;
  return command;
}

void linear_lift_update(LinearLift& mechanism, const LinearLiftCommand& command) {
  // 边缘触发：按一下切换方向状态
  if (command.toggle_up) {
    mechanism.state().open_loop_up = !mechanism.state().open_loop_up;
    mechanism.state().open_loop_down = false;
  }
  if (command.toggle_down) {
    mechanism.state().open_loop_down = !mechanism.state().open_loop_down;
    mechanism.state().open_loop_up = false;
  }

  const bool up = mechanism.state().open_loop_up;
  const bool down = mechanism.state().open_loop_down;

  if (!command.enabled || (!up && !down) || (up && down)) {
    stopcontrol(mechanism.lift_motor1(), vex::hold);
    stopcontrol(mechanism.lift_motor2(), vex::hold);
    refresh_state(mechanism);
    return;
  }

  const double speed = up ? +mechanism.config().open_loop_speed_pct
                          : -mechanism.config().open_loop_speed_pct;

  const auto& motor1_slot = mechanism.config().lift_motor1;
  const auto& motor2_slot = mechanism.config().lift_motor2;

  // 各电机独立限位：到限位就停
  double pos1 = get_position(mechanism.lift_motor1(), mechanism.config().position_units);
  if (speed > 0 && pos1 >= motor1_slot.position_max) {
    stopcontrol(mechanism.lift_motor1(), vex::hold);
  } else if (speed < 0 && pos1 <= motor1_slot.position_min) {
    stopcontrol(mechanism.lift_motor1(), vex::hold);
  } else {
    velocitycontrol(mechanism.lift_motor1(), speed, vex::pct);
  }

  double pos2 = get_position(mechanism.lift_motor2(), mechanism.config().position_units);
  if (speed > 0 && pos2 >= motor2_slot.position_max) {
    stopcontrol(mechanism.lift_motor2(), vex::hold);
  } else if (speed < 0 && pos2 <= motor2_slot.position_min) {
    stopcontrol(mechanism.lift_motor2(), vex::hold);
  } else {
    velocitycontrol(mechanism.lift_motor2(), speed, vex::pct);
  }

  refresh_state(mechanism);
}

void linear_lift_set_position(LinearLift& mechanism, double position) {
  mechanism.state().target_position = position;

  const auto& c = mechanism.config();
  const auto& motor1_slot = c.lift_motor1;
  const auto& motor2_slot = c.lift_motor2;

  const double pos1 = get_position(mechanism.lift_motor1(), c.position_units);
  const double pos2 = get_position(mechanism.lift_motor2(), c.position_units);
  const double current = pos1 + pos2;

  const bool moving_up = (position > current);

  // 各电机在运动方向上的剩余行程
  const double rem1 = moving_up ? motor1_slot.position_max - pos1
                                : pos1 - motor1_slot.position_min;
  const double rem2 = moving_up ? motor2_slot.position_max - pos2
                                : pos2 - motor2_slot.position_min;
  const double min_rem = std::max(0.0, std::min(rem1, rem2));

  // 两电机同步转动：各转 Δ，总和变化 = 2Δ，瓶颈电机最多转 min_rem
  double safe_target;
  if (moving_up) {
    safe_target = std::min(position, current + 2.0 * min_rem);
  } else {
    safe_target = std::max(position, current - 2.0 * min_rem);
  }

  // 总和位移平分到每个电机
  const double delta = (safe_target - current) / 2.0;

  double target1 = pos1 + delta;
  double target2 = pos2 + delta;
  // 绝对限位兜底
  target1 = std::max(motor1_slot.position_min, std::min(target1, motor1_slot.position_max));
  target2 = std::max(motor2_slot.position_min, std::min(target2, motor2_slot.position_max));

  mechanism.lift_motor1().spinToPosition(
      target1, c.position_units, c.closed_loop_speed_pct, vex::velocityUnits::pct, false);
  mechanism.lift_motor2().spinToPosition(
      target2, c.position_units, c.closed_loop_speed_pct, vex::velocityUnits::pct, false);

  refresh_state(mechanism);
}

void linear_lift_stop(LinearLift& mechanism, vex::brakeType brake_type) {
  mechanism.state() = LinearLiftState{};
  stopcontrol(mechanism.lift_motor1(), brake_type);
  stopcontrol(mechanism.lift_motor2(), brake_type);
}

LinearLiftState& linear_lift_state(LinearLift& mechanism) {
  return mechanism.state();
}

const LinearLiftState& linear_lift_state(const LinearLift& mechanism) {
  return mechanism.state();
}

}  // namespace basic::mechanism
