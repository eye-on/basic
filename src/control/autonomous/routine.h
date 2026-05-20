#ifndef BASIC_SRC_CONTROL_AUTONOMOUS_ROUTINE_H_
#define BASIC_SRC_CONTROL_AUTONOMOUS_ROUTINE_H_

#include "hardware/robot_hardware.h"
#include "hardware/robots/robot_state.h"

namespace basic::hardware::robots::autonomous {

enum class TravelDirection {
  kAuto,
  kForward,
  kReverse,
};

void update_upper_overhang_mode(
    RobotHardware& hardware,
    RobotState& state,
    bool wait_for_completion = true);

void update_middle_overhang_mode(
    RobotHardware& hardware,
    RobotState& state,
    bool wait_for_completion = true);

void partially_collapse_middle_overhang(
    RobotHardware& hardware,
    RobotState& state,
    bool wait_for_completion = true);

void update_under_overhang_mode(
    RobotHardware& hardware,
    RobotState& state,
    bool wait_for_completion = true);

void go_to_pose(
    RobotHardware& hardware,
    RobotState& state,
    vex::competition& competition,
    double target_x_mm,
    double target_y_mm,
    double target_heading_deg,
    TravelDirection travel_direction = TravelDirection::kAuto);

void drive_to_laser_distance_mm(
    RobotHardware& hardware,
    RobotState& state,
    vex::competition& competition,
    double target_distance_mm,
    double max_speed_pct = -1.0);

void run_routine(RobotHardware& hardware, RobotState& state, vex::competition& competition);

}  // namespace basic::hardware::robots::autonomous

#endif
