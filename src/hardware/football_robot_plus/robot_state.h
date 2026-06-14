#ifndef BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_ROBOT_STATE_H_
#define BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_ROBOT_STATE_H_

#include "hardware/football_robot_plus/vision.h"
#include "hardware/shared/state_types.h"

namespace basic::hardware::football_robot_plus {

struct RobotState {
  basic::hardware::shared::ControllerInputState controller;
  FootballVisionState vision;
};

}  // namespace basic::hardware::football_robot_plus

#endif  // BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_ROBOT_STATE_H_
