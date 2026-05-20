#ifndef BASIC_SRC_CONTROL_MECHANISMS_H_
#define BASIC_SRC_CONTROL_MECHANISMS_H_

#include "hardware/robot_hardware.h"
#include "hardware/robots/robot_state.h"

namespace basic::hardware::robots {

void mechanism_update(RobotHardware& hardware, RobotState& state);

void update_intake_mode(RobotHardware& hardware, RobotState& state);

void update_underthrow_mode(RobotHardware& hardware, RobotState& state);

void update_middlethrow_mode(RobotHardware& hardware, RobotState& state);

void update_upperthrow_mode(RobotHardware& hardware, RobotState& state);

void apply_mechanism_mode(RobotHardware& hardware, RobotState& state);

}  // namespace basic::hardware::robots

#endif
