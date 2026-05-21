#ifndef BASIC_SRC_INPUT_CONTROLLER_H_
#define BASIC_SRC_INPUT_CONTROLLER_H_

#include "hardware/robot_hardware.h"
#include "hardware/robots/robot_state.h"

namespace basic::hardware::robots {

void controller_update(RobotHardware& hardware, RobotState& state);

}  // namespace basic::hardware::robots

#endif
