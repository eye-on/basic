#ifndef BASIC_INCLUDE_CHASSIS_HEADING_HOLD_H_
#define BASIC_INCLUDE_CHASSIS_HEADING_HOLD_H_

#include "control/pid/controller.hpp"
#include "vex.h"

#include <cstdlib>

namespace basic::chassis {

struct HeadingHoldConfig {
  basic::control::pid::Pid::Config pid;
  int turn_deadzone{10};
};

struct HeadingHold {
  basic::control::pid::Pid pid;
  double target_heading_deg{0.0};
  bool locked{false};
  int turn_deadzone{10};
};

inline HeadingHold heading_hold_init(const HeadingHoldConfig& config = {}) {
  HeadingHold hold;
  hold.turn_deadzone = config.turn_deadzone;
  auto pid_cfg = config.pid;
  if (pid_cfg.kp == 0.0 && pid_cfg.ki == 0.0 && pid_cfg.kd == 0.0) {
    pid_cfg.kp = 0.2;
    pid_cfg.kd = 0.0;
    pid_cfg.out_min = -50.0;
    pid_cfg.out_max = 50.0;
  }
  hold.pid = basic::control::pid::Pid(pid_cfg);
  return hold;
}

inline double heading_hold_update(HeadingHold& hold,
                                  int raw_turn_input,
                                  double current_heading_deg) {
  if (std::abs(raw_turn_input) > hold.turn_deadzone) {
    hold.locked = false;
    hold.pid.reset();
    return 0.0;
  }

  if (!hold.locked) {
    hold.target_heading_deg = current_heading_deg;
    hold.locked = true;
    hold.pid.reset();
    return 0.0;
  }

  double measurement = current_heading_deg;
  while (measurement - hold.target_heading_deg > 180.0) measurement -= 360.0;
  while (measurement - hold.target_heading_deg < -180.0) measurement += 360.0;

  const auto result = hold.pid.update(hold.target_heading_deg, measurement);
  return result.ctrl;
}

}  // namespace basic::chassis

#endif  // BASIC_INCLUDE_CHASSIS_HEADING_HOLD_H_
