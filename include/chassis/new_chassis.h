#ifndef BASIC_INCLUDE_NEW_CHASSIS_H_
#define BASIC_INCLUDE_NEW_CHASSIS_H_

#include "chassis/steering/steering_drive.h"

namespace basic::chassis {

using NewChassis = basic::chassis::steering::SteeringDrive;
using NewChassisCommand = basic::chassis::steering::ArcadeDriveCommand;
using NewChassisConfig = basic::chassis::steering::SteeringDriveConfig;
using NewChassisState = basic::chassis::steering::SteeringDriveState;
using WheelUnit = basic::chassis::steering::WheelUnit;
using WheelUnitConfig = basic::chassis::steering::WheelUnitConfig;

inline NewChassis new_chassis_init(const NewChassisConfig& config) {
  return basic::chassis::steering::steering_init(config);
}

inline NewChassisCommand new_chassis_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input,
    vex::brakeType stop_brake_type = vex::coast) {
  const double vx_pct = static_cast<double>(
      basic::chassis::steering::controller_axis_value(
          input,
          basic::chassis::steering::ControllerAxis::kAxis2));
  const double vy_pct = static_cast<double>(
      basic::chassis::steering::controller_axis_value(
          input,
          basic::chassis::steering::ControllerAxis::kAxis1));
  const double omega_pct = static_cast<double>(
      basic::chassis::steering::controller_axis_value(
          input,
          basic::chassis::steering::ControllerAxis::kAxis4));

  return {vx_pct, vy_pct, omega_pct, stop_brake_type};
}

inline void new_chassis_update(
    NewChassis& chassis,
    const NewChassisCommand& command) {
  basic::chassis::steering::steering_update(chassis, command);
}

inline void new_chassis_stop(
    NewChassis& chassis,
    vex::brakeType brake_type = vex::coast) {
  NewChassisCommand stop_command{0, 0, 0, brake_type};
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

}  // namespace basic::chassis

#endif  // BASIC_INCLUDE_NEW_CHASSIS_H_
