#ifndef BASIC_SRC_HARDWARE_BED_ROBOT_STATE_H_
#define BASIC_SRC_HARDWARE_BED_ROBOT_STATE_H_

#include "hardware/shared/state_types.h"

namespace basic::hardware::bed {

struct RobotState {
  basic::hardware::shared::ControllerInputState controller;
};

}  // namespace basic::hardware::bed

#endif  // BASIC_SRC_HARDWARE_BED_ROBOT_STATE_H_
