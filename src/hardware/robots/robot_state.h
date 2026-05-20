#ifndef BASIC_SRC_HARDWARE_ROBOTS_ROBOT_STATE_H_
#define BASIC_SRC_HARDWARE_ROBOTS_ROBOT_STATE_H_

#include "vex.h"

namespace basic::hardware::robots {

enum class IndexedMechanismMode {
  kOff,
  kPreLoad,
  kLegacyIntake,
  kUnderTrow,
  kMiddleThrow,
  kUpperThrow,
  kSortIntake,
};

enum class OverhangMode{
  Collapse,
  Expansion,
};

struct ControllerInputState {
  int time_ms{0};

  int axis1{0};
  int axis2{0};
  int axis3{0};
  int axis4{0};

  int last_axis1{0};
  int last_axis2{0};
  int last_axis3{0};
  int last_axis4{0};

  bool l1{false};
  bool l2{false};
  bool r1{false};
  bool r2{false};
  bool x{false};
  bool y{false};
  bool a{false};
  bool b{false};
  bool left{false};
  bool right{false};
  bool up{false};
  bool down{false};

  bool last_l1{false};
  bool last_l2{false};
  bool last_r1{false};
  bool last_r2{false};
  bool last_x{false};
  bool last_y{false};
  bool last_a{false};
  bool last_b{false};
  bool last_left{false};
  bool last_right{false};
  bool last_up{false};
  bool last_down{false};

  bool press_x{false};
  bool press_y{false};
  bool press_a{false};
  bool press_b{false};
  bool press_up{false};
  bool press_down{false};
  bool press_left{false};
  bool press_right{false};
  bool press_l1{false};
  bool press_l2{false};
  bool press_r1{false};
  bool press_r2{false};

  double rating[4]{0, 0, 0, 0};
};

struct SensorState {
  bool initialized{false};
  int accelerate{0};
  char current_color_code{'N'};
  char previous_color_code{'N'};
  int last_update_ms{0};
  int hold_until_ms{0};
};

struct ChassisState {
  double fl{0};
  double fr{0};
  double bl{0};
  double br{0};
  vex::brakeType stop_brake_type{vex::coast};
};

struct MechanismState {
  IndexedMechanismMode indexed_mode{IndexedMechanismMode::kOff};
};

struct OverhangState {
  OverhangMode upper_overhang_mode{OverhangMode::Collapse};
  OverhangMode middle_overhang_mode{OverhangMode::Collapse};
  OverhangMode under_overhang_mode{OverhangMode::Collapse};
};

struct AutonomousState {
  bool initialized{false};
  double target_heading_deg{0.0};
  double estimated_heading_deg{0.0};
  double estimated_x_mm{0.0};
  double estimated_y_mm{0.0};
};

struct RobotState {
  ControllerInputState controller;
  SensorState sensors;
  ChassisState chassis;
  MechanismState mechanism;
  OverhangState overhang;
  AutonomousState autonomous;
  int blue_balls = 0;
  int red_balls = 0;
};

}  // namespace basic::hardware::robots

#endif
