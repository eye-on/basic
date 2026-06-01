#ifndef BASIC_INCLUDE_OLD_CHASSIS_H_
#define BASIC_INCLUDE_OLD_CHASSIS_H_

#include "chassis/arcade_drive.h"

namespace basic::chassis {

using OldChassis = ArcadeDrive<4, 4>;
using OldChassisCommand = ArcadeDriveCommand;
using OldChassisConfig = ArcadeDriveConfig<4, 4>;
using OldChassisState = ArcadeDriveState;

inline OldChassis old_chassis_init(const OldChassisConfig& config) {
  return arcade_init(config);
}

inline OldChassisCommand old_chassis_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input,
    vex::brakeType stop_brake_type = vex::coast) {
  return arcade_command_from_controller(
      input,
      ControllerAxis::kAxis2,
      ControllerAxis::kAxis4,
      stop_brake_type);
}

inline void old_chassis_update(
    OldChassis& chassis,
    const OldChassisCommand& command) {
  arcade_update(chassis, command);
}

inline void old_chassis_stop(
    OldChassis& chassis,
    vex::brakeType brake_type = vex::coast) {
  arcade_stop(chassis, brake_type);
}

inline OldChassisState& old_chassis_state(OldChassis& chassis) {
  return arcade_state(chassis);
}

inline const OldChassisState& old_chassis_state(const OldChassis& chassis) {
  return arcade_state(chassis);
}

inline std::array<vex::motor, 4>& old_chassis_left_motors(OldChassis& chassis) {
  return chassis.left_motors();
}

inline const std::array<vex::motor, 4>& old_chassis_left_motors(const OldChassis& chassis) {
  return chassis.left_motors();
}

inline std::array<vex::motor, 4>& old_chassis_right_motors(OldChassis& chassis) {
  return chassis.right_motors();
}

inline const std::array<vex::motor, 4>& old_chassis_right_motors(const OldChassis& chassis) {
  return chassis.right_motors();
}

}  // namespace basic::chassis

#endif
