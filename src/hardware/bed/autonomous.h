#ifndef BASIC_SRC_HARDWARE_BED_AUTONOMOUS_H_
#define BASIC_SRC_HARDWARE_BED_AUTONOMOUS_H_

#include "hardware/bed/robot_hardware.h"
#include "hardware/bed/robot_state.h"

namespace basic::hardware::bed::autonomous {

void run_routine(
    basic::hardware::bed::RobotHardware& hardware,
    basic::hardware::bed::RobotState& state,
    vex::competition& competition);

}  // namespace basic::hardware::bed::autonomous

#endif  // BASIC_SRC_HARDWARE_BED_AUTONOMOUS_H_
