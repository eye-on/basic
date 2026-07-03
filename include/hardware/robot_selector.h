#ifndef BASIC_HARDWARE_ROBOT_SELECTOR_H_
#define BASIC_HARDWARE_ROBOT_SELECTOR_H_

#include "app/robot.h"

namespace basic::hardware {

enum class RobotIdentity {
  kBasicRobot,
  kSecondRobot,
  kNewRobot,
  kFootballRobot,
};

inline constexpr RobotIdentity kSelectedRobot = RobotIdentity::kNewRobot;

basic::app::Robot& get_current_robot();

}  // namespace basic::hardware

#endif
