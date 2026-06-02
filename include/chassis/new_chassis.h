#ifndef BASIC_INCLUDE_NEW_CHASSIS_H_
#define BASIC_INCLUDE_NEW_CHASSIS_H_

#include "chassis/steering/steering_drive.h"

#include <cmath>

namespace basic::chassis {

using NewChassis = basic::chassis::steering::SteeringDrive;
using NewChassisCommand = basic::chassis::steering::ArcadeDriveCommand;
using NewChassisConfig = basic::chassis::steering::SteeringDriveConfig;
using NewChassisState = basic::chassis::steering::SteeringDriveState;
using WheelUnit = basic::chassis::steering::Wheel_Unit;
using WheelUnitConfig = basic::chassis::steering::Wheel_Unit_Config;
using PidConfig = basic::control::pid::Pid::Config;


inline NewChassis new_chassis_init(const NewChassisConfig& config) {
  return basic::chassis::steering::steering_init(config);
}

inline NewChassisCommand new_chassis_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input,
    vex::brakeType stop_brake_type = vex::coast) {
  const int axis1 = basic::chassis::steering::controller_axis_value(input, basic::chassis::steering::ControllerAxis::kAxis1);
  const int axis2 = basic::chassis::steering::controller_axis_value(input, basic::chassis::steering::ControllerAxis::kAxis2);

  const int velocity = axis1;
  const double heading_deg = std::atan2(static_cast<double>(axis2), static_cast<double>(axis1)) * (180.0 / M_PI);
  return {
      velocity,
      static_cast<int>(heading_deg),
      stop_brake_type,
  };
}

inline void new_chassis_update(
    NewChassis& chassis,
    const NewChassisCommand& command) {
  basic::chassis::steering::steering_update(chassis, command);
}

inline void new_chassis_stop(
    NewChassis& chassis,
    vex::brakeType brake_type = vex::coast) {
  NewChassisCommand stop_command{0, 0, brake_type};
  new_chassis_update(chassis, stop_command);
}

inline NewChassisState& new_chassis_state(NewChassis& chassis) {
  return chassis.state();
}

inline const NewChassisState& new_chassis_state(const NewChassis& chassis) {
  return chassis.state();
}

inline WheelUnit& new_chassis_fr(NewChassis& chassis) {
  return chassis.fr();
}

inline const WheelUnit& new_chassis_fr(const NewChassis& chassis) {
  return chassis.fr();
}

inline WheelUnit& new_chassis_fl(NewChassis& chassis) {
  return chassis.fl();
}

inline const WheelUnit& new_chassis_fl(const NewChassis& chassis) {
  return chassis.fl();
}

inline WheelUnit& new_chassis_br(NewChassis& chassis) {
  return chassis.br();
}

inline const WheelUnit& new_chassis_br(const NewChassis& chassis) {
  return chassis.br();
}

inline WheelUnit& new_chassis_bl(NewChassis& chassis) {
  return chassis.bl();
}

inline const WheelUnit& new_chassis_bl(const NewChassis& chassis) {
  return chassis.bl();
}

inline basic::control::pid::Pid& new_chassis_velocity_pid(NewChassis& chassis) {
  return chassis.velocity_pid();
}

inline const basic::control::pid::Pid& new_chassis_velocity_pid(const NewChassis& chassis) {
  return chassis.velocity_pid();
}

inline basic::control::pid::Pid& new_chassis_heading_pid(NewChassis& chassis) {
  return chassis.heading_pid();
}

inline const basic::control::pid::Pid& new_chassis_heading_pid(const NewChassis& chassis) {
  return chassis.heading_pid();
}

}  // namespace basic::chassis

#endif  // BASIC_INCLUDE_NEW_CHASSIS_H_
