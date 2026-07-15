#ifndef BASIC_SRC_HARDWARE_LOOKLOOK_ROBOT_STATE_H_
#define BASIC_SRC_HARDWARE_LOOKLOOK_ROBOT_STATE_H_

#include "hardware/shared/state_types.h"
#include "mechanism/linear_lift.h"

namespace basic::hardware::looklook {

struct RobotState {
  basic::hardware::shared::ControllerInputState controller;
  basic::mechanism::LinearLiftState lift;
};

}  // namespace basic::hardware::looklook

#endif  // BASIC_SRC_HARDWARE_LOOKLOOK_ROBOT_STATE_H_
