#ifndef BASIC_SRC_HARDWARE_NEW_ROBOT_ROBOT_STATE_H_
#define BASIC_SRC_HARDWARE_NEW_ROBOT_ROBOT_STATE_H_

#include "hardware/shared/state_types.h"

namespace basic::hardware::new_robot {

struct RobotState {
  basic::hardware::shared::ControllerInputState controller;
  basic::hardware::shared::SensorState sensors;
  basic::hardware::shared::AutonomousState autonomous;
};

}  // namespace basic::hardware::new_robot

#endif  // BASIC_SRC_HARDWARE_NEW_ROBOT_ROBOT_STATE_H_
