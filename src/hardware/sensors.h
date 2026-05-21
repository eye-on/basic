#ifndef BASIC_SRC_HARDWARE_SENSORS_H_
#define BASIC_SRC_HARDWARE_SENSORS_H_

#include "hardware/robot_hardware.h"
#include "hardware/robots/robot_state.h"

namespace basic::hardware::robots {

void sensor_update(RobotHardware& hardware, RobotState& state, const vex::color target, const int wait_frames=5, const int continuous_fps=30);

void count_balls_number(RobotHardware& hardware, RobotState& state);

}  // namespace basic::hardware::robots

#endif
