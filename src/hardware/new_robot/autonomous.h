#ifndef BASIC_SRC_HARDWARE_NEW_ROBOT_AUTONOMOUS_H_
#define BASIC_SRC_HARDWARE_NEW_ROBOT_AUTONOMOUS_H_

#include "hardware/new_robot/robot_hardware.h"
#include "hardware/new_robot/robot_state.h"
#include "vex.h"

namespace basic::hardware::new_robot::autonomous {

void run_routine(RobotHardware& hardware, RobotState& state, vex::competition& competition);

}  // namespace basic::hardware::new_robot::autonomous

#endif  // BASIC_SRC_HARDWARE_NEW_ROBOT_AUTONOMOUS_H_
