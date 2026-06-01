#ifndef BASIC_SRC_HARDWARE_SECOND_ROBOT_AUTONOMOUS_H_
#define BASIC_SRC_HARDWARE_SECOND_ROBOT_AUTONOMOUS_H_

#include "hardware/second_robot/robot_hardware.h"
#include "hardware/second_robot/robot_state.h"

namespace basic::hardware::second_robot::autonomous {

enum class TravelDirection {
  kAuto,
  kForward,
  kReverse,
};

void drive_distance_mm(
    basic::hardware::second_robot::RobotHardware& hardware,
    basic::hardware::second_robot::RobotState& state,
    vex::competition& competition,
    double distance_mm,
    double max_speed_pct);

void turn_to_heading_deg(
    basic::hardware::second_robot::RobotHardware& hardware,
    basic::hardware::second_robot::RobotState& state,
    vex::competition& competition,
    double target_heading_deg,
    double max_turn_speed_pct);

void run_routine(
    basic::hardware::second_robot::RobotHardware& hardware,
    basic::hardware::second_robot::RobotState& state,
    vex::competition& competition);

}  // namespace basic::hardware::second_robot::autonomous

#endif
