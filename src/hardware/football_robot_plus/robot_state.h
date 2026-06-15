#ifndef BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_ROBOT_STATE_H_
#define BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_ROBOT_STATE_H_

#include "hardware/football_robot_plus/vision.h"
#include "hardware/shared/state_types.h"
#include "mechanism/camera_gimbal.h"
#include "mechanism/dual_motor_actuator.h"

namespace basic::hardware::football_robot_plus {

struct RobotState {
  basic::hardware::shared::ControllerInputState controller;
  FootballVisionState vision;
  basic::mechanism::CameraGimbalState camera_gimbal;
  basic::mechanism::DualMotorActuatorState dual_motor_actuator;
  basic::hardware::shared::AutonomousState autonomous;
};

}  // namespace basic::hardware::football_robot_plus

#endif  // BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_ROBOT_STATE_H_
