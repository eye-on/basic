#ifndef BASIC_SRC_CONTROL_SECOND_ROBOT_MECHANISMS_H_
#define BASIC_SRC_CONTROL_SECOND_ROBOT_MECHANISMS_H_

#include "hardware/second_robot/robot_hardware.h"
#include "hardware/second_robot/robot_state.h"

namespace basic::control::second_robot {

void mechanism_update(
    basic::hardware::second_robot::RobotHardware& hardware,
    basic::hardware::second_robot::RobotState& state);

void set_shooter_mode(
    basic::hardware::second_robot::RobotHardware& hardware,
    basic::hardware::second_robot::RobotState& state,
    basic::hardware::second_robot::ShooterMode mode,
    double speed_pct);

}  // namespace basic::control::second_robot

#endif
