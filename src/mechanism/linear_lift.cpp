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

/// 计算限位减速系数：距限位越近速度越小，0=停止，1=全速
double decel_factor(double position, double limit_min, double limit_max,
                    double threshold, bool moving_up) {
  if (threshold <= 0.0) return 1.0;
  if (moving_up) {
    double dist = limit_max - position;
    if (dist <= 0.0) return 0.0;
    if (dist >= threshold) return 1.0;
    return dist / threshold;
  } else {
    double dist = position - limit_min;
    if (dist <= 0.0) return 0.0;
    if (dist >= threshold) return 1.0;
    return dist / threshold;
  }
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
  const auto stop_brake = mechanism.config().stop_brake_type;

  if (!command.enabled || (!up && !down) || (up && down)) {
    stopcontrol(mechanism.lift_motor1(), stop_brake);
    stopcontrol(mechanism.lift_motor2(), stop_brake);
    refresh_state(mechanism);
    return;
  }

  const double speed = up ? +mechanism.config().open_loop_speed_pct
                          : -mechanism.config().open_loop_speed_down_pct;

  const auto& motor1_slot = mechanism.config().lift_motor1;
  const auto& motor2_slot = mechanism.config().lift_motor2;

  // 各电机独立限位 + 减速区
  const double threshold = mechanism.config().decel_threshold;
  double pos1 = get_position(mechanism.lift_motor1(), mechanism.config().position_units);
  double pos2 = get_position(mechanism.lift_motor2(), mechanism.config().position_units);

  const double factor1 = decel_factor(pos1, motor1_slot.position_min, motor1_slot.position_max, threshold, up);
  const double factor2 = decel_factor(pos2, motor2_slot.position_min, motor2_slot.position_max, threshold, up);

  const double min_speed = mechanism.config().decel_min_speed_pct;

  if (factor1 <= 0.0) {
    stopcontrol(mechanism.lift_motor1(), stop_brake);
  } else {
    const double raw = speed * factor1;
    const double clamped = (raw > 0) ? std::max(raw, min_speed) : std::min(raw, -min_speed);
    velocitycontrol(mechanism.lift_motor1(), clamped, vex::pct);
  }

  if (factor2 <= 0.0) {
    stopcontrol(mechanism.lift_motor2(), stop_brake);
  } else {
    const double raw = speed * factor2;
    const double clamped = (raw > 0) ? std::max(raw, min_speed) : std::min(raw, -min_speed);
    velocitycontrol(mechanism.lift_motor2(), clamped, vex::pct);
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
  const double closed_loop_speed =
      moving_up ? c.closed_loop_speed_pct : c.closed_loop_speed_down_pct;

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

  // 按行程比例分配速度：行程长的电机更快，保证同时到达
  const double range1 = motor1_slot.position_max - motor1_slot.position_min;
  const double range2 = motor2_slot.position_max - motor2_slot.position_min;
  const double max_range = std::max(range1, range2);
  const double speed1 = max_range > 0.0 ? closed_loop_speed * (range1 / max_range) : closed_loop_speed;
  const double speed2 = max_range > 0.0 ? closed_loop_speed * (range2 / max_range) : closed_loop_speed;

  mechanism.lift_motor1().spinToPosition(
      target1, c.position_units, speed1, vex::velocityUnits::pct, false);
  mechanism.lift_motor2().spinToPosition(
      target2, c.position_units, speed2, vex::velocityUnits::pct, false);

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
