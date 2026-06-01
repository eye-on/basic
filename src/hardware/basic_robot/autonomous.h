#ifndef BASIC_SRC_HARDWARE_BASIC_ROBOT_AUTONOMOUS_H_
#define BASIC_SRC_HARDWARE_BASIC_ROBOT_AUTONOMOUS_H_

#include "hardware/basic_robot/robot_hardware.h"
#include "hardware/basic_robot/robot_state.h"

namespace basic::hardware::basic_robot::autonomous {

enum class TravelDirection {
  kAuto,
  kForward,
  kReverse,
};

void go_to_pose(
    basic::hardware::basic_robot::RobotHardware& hardware,
    basic::hardware::basic_robot::RobotState& state,
    vex::competition& competition,
    double target_x_mm,
    double target_y_mm,
    double target_heading_deg,
    TravelDirection travel_direction = TravelDirection::kAuto);

void drive_to_laser_distance_mm(
    basic::hardware::basic_robot::RobotHardware& hardware,
    basic::hardware::basic_robot::RobotState& state,
    vex::competition& competition,
    double target_distance_mm,
    double max_speed_pct = -1.0);

void run_routine(
    basic::hardware::basic_robot::RobotHardware& hardware,
    basic::hardware::basic_robot::RobotState& state,
    vex::competition& competition);

}  // namespace basic::hardware::basic_robot::autonomous

#endif
