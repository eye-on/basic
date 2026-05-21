#ifndef BASIC_SRC_CONTROL_CHASSIS_H_
#define BASIC_SRC_CONTROL_CHASSIS_H_

#include "hardware/robot_hardware.h"
#include "hardware/robots/robot_state.h"

namespace basic::hardware::robots {

void chassis_update(RobotHardware& hardware, RobotState& state);

}  // namespace basic::hardware::robots

#endif
