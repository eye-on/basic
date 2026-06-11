#ifndef BASIC_INCLUDE_NEW_CHASSIS_H_
#define BASIC_INCLUDE_NEW_CHASSIS_H_

#include "chassis/h_drive.h"

namespace basic::chassis {

using NewChassis = HDrive<2, 2>;
using NewChassisCommand = HDriveCommand;
using NewChassisConfig = HDriveConfig<2, 2>;
using NewChassisState = HDriveState;

inline NewChassis new_chassis_init(const NewChassisConfig& config) {
  return h_drive_init(config);
}

inline NewChassisCommand new_chassis_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input,
    vex::brakeType stop_brake_type = vex::coast) {
  return h_drive_command_from_controller(
      input,
      ControllerAxis::kAxis2,
      ControllerAxis::kAxis1,
      ControllerAxis::kAxis4,
      stop_brake_type);
}

inline void new_chassis_update(
    NewChassis& chassis,
    const NewChassisCommand& command) {
  h_drive_update(chassis, command);
}

inline void new_chassis_stop(
    NewChassis& chassis,
    vex::brakeType brake_type = vex::coast) {
  h_drive_stop(chassis, brake_type);
}

inline NewChassisState& new_chassis_state(NewChassis& chassis) {
  return h_drive_state(chassis);
}

inline const NewChassisState& new_chassis_state(const NewChassis& chassis) {
  return h_drive_state(chassis);
}

inline std::array<vex::motor, 2>& new_chassis_left_motors(NewChassis& chassis) {
  return chassis.left_motors();
}

inline const std::array<vex::motor, 2>& new_chassis_left_motors(const NewChassis& chassis) {
  return chassis.left_motors();
}

inline std::array<vex::motor, 2>& new_chassis_right_motors(NewChassis& chassis) {
  return chassis.right_motors();
}

inline const std::array<vex::motor, 2>& new_chassis_right_motors(const NewChassis& chassis) {
  return chassis.right_motors();
}

inline vex::motor& new_chassis_center_motor(NewChassis& chassis) {
  return chassis.center_motor();
}

inline const vex::motor& new_chassis_center_motor(const NewChassis& chassis) {
  return chassis.center_motor();
}

inline vex::motor& new_chassis_fr_motor(NewChassis& chassis) {
  return chassis.right_motors()[0];
}

inline const vex::motor& new_chassis_fr_motor(const NewChassis& chassis) {
  return chassis.right_motors()[0];
}

inline vex::motor& new_chassis_br_motor(NewChassis& chassis) {
  return chassis.right_motors()[1];
}

inline const vex::motor& new_chassis_br_motor(const NewChassis& chassis) {
  return chassis.right_motors()[1];
}

inline vex::motor& new_chassis_fl_motor(NewChassis& chassis) {
  return chassis.left_motors()[0];
}

inline const vex::motor& new_chassis_fl_motor(const NewChassis& chassis) {
  return chassis.left_motors()[0];
}

inline vex::motor& new_chassis_bl_motor(NewChassis& chassis) {
  return chassis.left_motors()[1];
}

inline const vex::motor& new_chassis_bl_motor(const NewChassis& chassis) {
  return chassis.left_motors()[1];
}

inline vex::motor& new_chassis_fr(NewChassis& chassis) {
  return new_chassis_fr_motor(chassis);
}

inline const vex::motor& new_chassis_fr(const NewChassis& chassis) {
  return new_chassis_fr_motor(chassis);
}

inline vex::motor& new_chassis_fl(NewChassis& chassis) {
  return new_chassis_fl_motor(chassis);
}

inline const vex::motor& new_chassis_fl(const NewChassis& chassis) {
  return new_chassis_fl_motor(chassis);
}

inline vex::motor& new_chassis_br(NewChassis& chassis) {
  return new_chassis_br_motor(chassis);
}

inline const vex::motor& new_chassis_br(const NewChassis& chassis) {
  return new_chassis_br_motor(chassis);
}

inline vex::motor& new_chassis_bl(NewChassis& chassis) {
  return new_chassis_bl_motor(chassis);
}

inline const vex::motor& new_chassis_bl(const NewChassis& chassis) {
  return new_chassis_bl_motor(chassis);
}

}  // namespace basic::chassis

#endif  // BASIC_INCLUDE_NEW_CHASSIS_H_
