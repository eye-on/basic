#ifndef BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_ROBOT_STATE_H_
#define BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_ROBOT_STATE_H_

#include "hardware/shared/state_types.h"
#include "mechanism/single_pneumatic.h"

namespace basic::hardware::football_robot {

struct RobotState {
  basic::hardware::shared::ControllerInputState controller;
  basic::mechanism::SinglePneumaticState actuator;
};

}  // namespace basic::hardware::football_robot

#endif  // BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_ROBOT_STATE_H_
