#ifndef BASIC_SRC_CONTROL_SECOND_ROBOT_CHASSIS_H_
#define BASIC_SRC_CONTROL_SECOND_ROBOT_CHASSIS_H_

#include "hardware/second_robot/robot_hardware.h"
#include "hardware/second_robot/robot_state.h"

namespace basic::control::second_robot {

void chassis_update(
    basic::hardware::second_robot::RobotHardware& hardware,
    basic::hardware::second_robot::RobotState& state);

}  // namespace basic::control::second_robot

#endif
