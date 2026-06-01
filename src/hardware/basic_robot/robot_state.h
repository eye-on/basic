#ifndef BASIC_SRC_HARDWARE_BASIC_ROBOT_ROBOT_STATE_H_
#define BASIC_SRC_HARDWARE_BASIC_ROBOT_ROBOT_STATE_H_

#include "hardware/shared/state_types.h"

namespace basic::hardware::basic_robot {

struct RobotState {
  basic::hardware::shared::ControllerInputState controller;
  basic::hardware::shared::SensorState sensors;
  basic::hardware::shared::AutonomousState autonomous;
};

}  // namespace basic::hardware::basic_robot

#endif
