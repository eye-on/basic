#ifndef BASIC_SRC_CONTROL_MECHANISMS_H_
#define BASIC_SRC_CONTROL_MECHANISMS_H_

#include "hardware/robot_hardware.h"
#include "hardware/robots/robot_state.h"

namespace basic::hardware::robots {

void mechanism_update(RobotHardware& hardware, RobotState& state);

void disable_indexed_mode(RobotHardware& hardware, RobotState& state);

void enable_preload_mode(RobotHardware& hardware, RobotState& state);

void enable_intake_mode(RobotHardware& hardware, RobotState& state);

void enable_underthrow_mode(RobotHardware& hardware, RobotState& state);

void enable_middlethrow_mode(RobotHardware& hardware, RobotState& state);

void enable_upperthrow_mode(RobotHardware& hardware, RobotState& state);

void apply_indexed_mode(RobotHardware& hardware, RobotState& state);

}  // namespace basic::hardware::robots

#endif
