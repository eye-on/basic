#ifndef BASIC_INCLUDE_H_CHASSIS_H_
#define BASIC_INCLUDE_H_CHASSIS_H_

#include "chassis/h_drive.h"

namespace basic::chassis {

using HChassis = HDrive<2, 2>;
using HChassisCommand = HDriveCommand;
using HChassisConfig = HDriveConfig<2, 2>;
using HChassisState = HDriveState;

inline HChassis h_chassis_init(const HChassisConfig& config) {
  return h_drive_init(config);
}

inline HChassisCommand h_chassis_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input,
    vex::brakeType stop_brake_type = vex::coast) {
  return h_drive_command_from_controller(
      input,
      ControllerAxis::kAxis2,
      ControllerAxis::kAxis1,
      ControllerAxis::kAxis4,
      stop_brake_type);
}

inline void h_chassis_update(
    HChassis& chassis,
    const HChassisCommand& command) {
  h_drive_update(chassis, command);
}

inline void h_chassis_stop(
    HChassis& chassis,
    vex::brakeType brake_type = vex::coast) {
  h_drive_stop(chassis, brake_type);
}

inline HChassisState& h_chassis_state(HChassis& chassis) {
  return h_drive_state(chassis);
}

inline const HChassisState& h_chassis_state(const HChassis& chassis) {
  return h_drive_state(chassis);
}

inline std::array<vex::motor, 2>& h_chassis_left_motors(HChassis& chassis) {
  return chassis.left_motors();
}

inline const std::array<vex::motor, 2>& h_chassis_left_motors(
    const HChassis& chassis) {
  return chassis.left_motors();
}

inline std::array<vex::motor, 2>& h_chassis_right_motors(HChassis& chassis) {
  return chassis.right_motors();
}

inline const std::array<vex::motor, 2>& h_chassis_right_motors(
    const HChassis& chassis) {
  return chassis.right_motors();
}

inline vex::motor& h_chassis_center_motor(HChassis& chassis) {
  return chassis.center_motor();
}

inline const vex::motor& h_chassis_center_motor(const HChassis& chassis) {
  return chassis.center_motor();
}

inline vex::motor& h_chassis_fr_motor(HChassis& chassis) {
  return chassis.right_motors()[0];
}

inline const vex::motor& h_chassis_fr_motor(const HChassis& chassis) {
  return chassis.right_motors()[0];
}

inline vex::motor& h_chassis_br_motor(HChassis& chassis) {
  return chassis.right_motors()[1];
}

inline const vex::motor& h_chassis_br_motor(const HChassis& chassis) {
  return chassis.right_motors()[1];
}

inline vex::motor& h_chassis_fl_motor(HChassis& chassis) {
  return chassis.left_motors()[0];
}

inline const vex::motor& h_chassis_fl_motor(const HChassis& chassis) {
  return chassis.left_motors()[0];
}

inline vex::motor& h_chassis_bl_motor(HChassis& chassis) {
  return chassis.left_motors()[1];
}

inline const vex::motor& h_chassis_bl_motor(const HChassis& chassis) {
  return chassis.left_motors()[1];
}

inline vex::motor& h_chassis_fr(HChassis& chassis) {
  return h_chassis_fr_motor(chassis);
}

inline const vex::motor& h_chassis_fr(const HChassis& chassis) {
  return h_chassis_fr_motor(chassis);
}

inline vex::motor& h_chassis_fl(HChassis& chassis) {
  return h_chassis_fl_motor(chassis);
}

inline const vex::motor& h_chassis_fl(const HChassis& chassis) {
  return h_chassis_fl_motor(chassis);
}

inline vex::motor& h_chassis_br(HChassis& chassis) {
  return h_chassis_br_motor(chassis);
}

inline const vex::motor& h_chassis_br(const HChassis& chassis) {
  return h_chassis_br_motor(chassis);
}

inline vex::motor& h_chassis_bl(HChassis& chassis) {
  return h_chassis_bl_motor(chassis);
}

inline const vex::motor& h_chassis_bl(const HChassis& chassis) {
  return h_chassis_bl_motor(chassis);
}

}  // namespace basic::chassis

#endif  // BASIC_INCLUDE_H_CHASSIS_H_
