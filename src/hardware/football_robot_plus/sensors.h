#ifndef BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_SENSORS_H_
#define BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_SENSORS_H_

#include "hardware/football_robot_plus/robot_hardware.h"
#include "hardware/football_robot_plus/robot_state.h"
#include "hardware/football_robot_plus/runtime.h"

namespace basic::hardware::football_robot_plus::sensors {

FootballVisionConfig default_vision_config_for_sensor();

void configure_vision(
    RobotHardware& hardware,
    RobotState& state,
    RuntimeState& runtime,
    const FootballVisionConfig& config);

void set_vision_target_color(
    RobotState& state,
    basic::identify::VisionTargetColor color);

basic::vision::EstimateResult submit_yolo_detection(
    RobotHardware& hardware,
    RobotState& state,
    RuntimeState& runtime,
    const YoloDetection& detection);

void clear_yolo_detection(RobotHardware& hardware, RobotState& state);
void refresh_camera_gimbal_state(RobotHardware& hardware, RobotState& state);
void refresh_dual_motor_actuator_state(RobotHardware& hardware, RobotState& state);
void update(RobotHardware& hardware, RobotState& state, RuntimeState& runtime);

}  // namespace basic::hardware::football_robot_plus::sensors

#endif  // BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_SENSORS_H_
