#ifndef BASIC_INCLUDE_SECOND_CHASSIS_H_
#define BASIC_INCLUDE_SECOND_CHASSIS_H_

#include "chassis/arcade_drive.h"

namespace basic::chassis {

using SecondChassis = ArcadeDrive<3, 3>;
using SecondChassisCommand = ArcadeDriveCommand;
using SecondChassisConfig = ArcadeDriveConfig<3, 3>;
using SecondChassisState = ArcadeDriveState;

inline SecondChassis second_chassis_init(const SecondChassisConfig& config) {
  return arcade_init(config);
}

inline SecondChassisCommand second_chassis_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input,
    vex::brakeType stop_brake_type = vex::coast) {
  return arcade_command_from_controller(
      input,
      ControllerAxis::kAxis3,
      ControllerAxis::kAxis1,
      stop_brake_type);
}

inline void second_chassis_update(
    SecondChassis& chassis,
    const SecondChassisCommand& command) {
  arcade_update(chassis, command);
}

inline void second_chassis_stop(
    SecondChassis& chassis,
    vex::brakeType brake_type = vex::coast) {
  arcade_stop(chassis, brake_type);
}

inline SecondChassisState& second_chassis_state(SecondChassis& chassis) {
  return arcade_state(chassis);
}

inline const SecondChassisState& second_chassis_state(const SecondChassis& chassis) {
  return arcade_state(chassis);
}

inline std::array<vex::motor, 3>& second_chassis_left_motors(SecondChassis& chassis) {
  return chassis.left_motors();
}

inline const std::array<vex::motor, 3>& second_chassis_left_motors(const SecondChassis& chassis) {
  return chassis.left_motors();
}

inline std::array<vex::motor, 3>& second_chassis_right_motors(SecondChassis& chassis) {
  return chassis.right_motors();
}

inline const std::array<vex::motor, 3>& second_chassis_right_motors(const SecondChassis& chassis) {
  return chassis.right_motors();
}

}  // namespace basic::chassis

#endif
