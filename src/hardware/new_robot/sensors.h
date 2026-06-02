#ifndef BASIC_SRC_HARDWARE_NEW_ROBOT_SENSORS_H_
#define BASIC_SRC_HARDWARE_NEW_ROBOT_SENSORS_H_

#include "hardware/new_robot/robot_hardware.h"
#include "hardware/new_robot/robot_state.h"

namespace basic::hardware::new_robot {

void sensor_update(RobotHardware& hardware, RobotState& state);

}  // namespace basic::hardware::new_robot

#endif  // BASIC_SRC_HARDWARE_NEW_ROBOT_SENSORS_H_
