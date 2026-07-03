#include "hardware/football_robot_plus/sensors.h"

namespace basic::hardware::football_robot_plus::sensors {

namespace {

constexpr double kEstimatedPositionYOffsetMm = 30.0;
constexpr int kExternalVisionStaleTimeoutMs = 500;

bool has_fresh_external_detection(const FootballVisionState& vision, int now_ms) {
  if (!vision.external_link.online || !vision.last_detection.has_detection) {
    return false;
  }
  if (now_ms < vision.last_detection_rx_time_ms) {
    return false;
  }
  return now_ms - vision.last_detection_rx_time_ms <= kExternalVisionStaleTimeoutMs;
}

}  // namespace

FootballVisionConfig default_vision_config_for_sensor() {
  FootballVisionConfig config;
  config.image_width_px = 640.0;
  config.image_height_px = 480.0;
  config.camera.fx = 660.037869;
  config.camera.fy = 932.486763;
  config.camera.cx = 332.090692;
  config.camera.cy = 91.419530;
  config.camera.k1 = 0.392595721;
  config.camera.k2 = -0.282416438;
  config.camera.p1 = -0.128963182;
  config.camera.p2 = 0.005571122;
  config.camera.k3 = 0.517875520;
  config.football_diameter_mm = kDefaultFootballDiameterMm;
  config.expected_class_id = -1;
  config.camera_extrinsics.x_mm = -50.0;
  config.camera_extrinsics.y_mm = 0.0;
  config.camera_extrinsics.z_mm = 150.0;
  config.camera_extrinsics.roll_deg = 0.0;
  config.camera_extrinsics.pitch_deg = 0.0;
  config.camera_extrinsics.yaw_deg = 0.0;
  return config;
}

void configure_vision(
    RobotHardware& hardware,
    RobotState& state,
    RuntimeState& runtime,
    const FootballVisionConfig& config) {
  const basic::identify::VisionTargetColor target_color = state.vision.target_color;
  const basic::identify::LargestBlobDetection last_blob_detection =
      state.vision.last_blob_detection;
  const YoloDetection last_detection = state.vision.last_detection;
  const basic::vision::EstimateResult last_estimate = state.vision.last_estimate;
  const bool estimate_available = state.vision.estimate_available;
  const bool class_filter_passed = state.vision.class_filter_passed;
  const int last_detection_rx_time_ms = state.vision.last_detection_rx_time_ms;
  const ExternalVisionLinkState external_link = state.vision.external_link;

  state.vision = FootballVisionState{};
  state.vision.config = config;
  state.vision.target_color = target_color;
  state.vision.last_blob_detection = last_blob_detection;
  state.vision.last_blob_detection.color = target_color;
  state.vision.last_detection = last_detection;
  state.vision.last_estimate = last_estimate;
  state.vision.estimate_available = estimate_available;
  state.vision.class_filter_passed = class_filter_passed;
  state.vision.last_detection_rx_time_ms = last_detection_rx_time_ms;
  state.vision.external_link = external_link;
  runtime.locator.set_config(config.estimator);
  state.vision.last_update_time_ms = hardware.brain.timer(vex::timeUnits::msec);
}

void set_vision_target_color(
    RobotState& state,
    basic::identify::VisionTargetColor color) {
  state.vision.target_color = color;
  state.vision.last_blob_detection.color = color;
  state.vision.external_link.reported_color_code = basic::identify::color_code(color);
}

basic::vision::EstimateResult submit_yolo_detection(
    RobotHardware& hardware,
    RobotState& state,
    RuntimeState& runtime,
    const YoloDetection& detection) {
  state.vision.last_blob_detection.sensor_installed = state.vision.external_link.online;
  state.vision.last_blob_detection.has_detection = detection.has_detection;
  state.vision.last_blob_detection.signature_id = detection.class_id;
  state.vision.last_blob_detection.object_count = detection.has_detection ? 1 : 0;
  state.vision.last_blob_detection.origin_x_px = static_cast<int>(detection.bbox_px.x);
  state.vision.last_blob_detection.origin_y_px = static_cast<int>(detection.bbox_px.y);
  state.vision.last_blob_detection.center_x_px =
      static_cast<int>(detection.bbox_px.x + detection.bbox_px.width * 0.5);
  state.vision.last_blob_detection.center_y_px =
      static_cast<int>(detection.bbox_px.y + detection.bbox_px.height * 0.5);
  state.vision.last_blob_detection.width_px = static_cast<int>(detection.bbox_px.width);
  state.vision.last_blob_detection.height_px = static_cast<int>(detection.bbox_px.height);
  state.vision.last_blob_detection.image_width_px = detection.image_width_px;
  state.vision.last_blob_detection.image_height_px = detection.image_height_px;
  state.vision.last_detection = detection;
  state.vision.class_filter_passed =
      !detection.has_detection || state.vision.config.expected_class_id < 0 ||
      detection.class_id == state.vision.config.expected_class_id;
  state.vision.last_update_time_ms = hardware.brain.timer(vex::timeUnits::msec);
  state.vision.last_detection_rx_time_ms = state.vision.last_update_time_ms;

  if (!detection.has_detection) {
    state.vision.last_estimate = basic::vision::EstimateResult{};
    state.vision.estimate_available = false;
    return state.vision.last_estimate;
  }

  state.vision.last_estimate =
      estimate_football_from_yolo(runtime.locator, state.vision.config, detection);
  if (state.vision.last_estimate.valid) {
    state.vision.last_estimate.position_camera_mm.y -= kEstimatedPositionYOffsetMm;
  }
  state.vision.estimate_available = true;
  return state.vision.last_estimate;
}

void clear_yolo_detection(RobotHardware& hardware, RobotState& state) {
  state.vision.last_blob_detection.has_detection = false;
  state.vision.last_blob_detection.object_count = 0;
  state.vision.last_blob_detection.signature_id = 0;
  state.vision.last_blob_detection.origin_x_px = 0;
  state.vision.last_blob_detection.origin_y_px = 0;
  state.vision.last_blob_detection.center_x_px = 0;
  state.vision.last_blob_detection.center_y_px = 0;
  state.vision.last_blob_detection.width_px = 0;
  state.vision.last_blob_detection.height_px = 0;
  state.vision.last_detection = YoloDetection{};
  state.vision.last_estimate = basic::vision::EstimateResult{};
  state.vision.estimate_available = false;
  state.vision.class_filter_passed = true;
  state.vision.last_update_time_ms = hardware.brain.timer(vex::timeUnits::msec);
}

void refresh_camera_gimbal_state(RobotHardware& hardware, RobotState& state) {
  basic::mechanism::camera_gimbal_refresh_state(hardware.camera_gimbal);
  state.camera_gimbal = basic::mechanism::camera_gimbal_state(hardware.camera_gimbal);
}

void refresh_dual_motor_actuator_state(RobotHardware& hardware, RobotState& state) {
  basic::mechanism::dual_motor_actuator_refresh_state(hardware.dual_motor_actuator);
  state.dual_motor_actuator =
      basic::mechanism::dual_motor_actuator_state(hardware.dual_motor_actuator);
}

void update(RobotHardware& hardware, RobotState& state, RuntimeState& runtime) {
  refresh_dual_motor_actuator_state(hardware, state);

  ExternalVisionPacket packet;
  const bool any_update = hardware.external_vision.poll(&packet);
  state.vision.external_link = hardware.external_vision.link_state();
  state.vision.last_blob_detection.sensor_installed = state.vision.external_link.online;

  if (packet.has_observation_update) {
    set_vision_target_color(state, packet.target_color);
    submit_yolo_detection(hardware, state, runtime, packet.detection);
  } else if (!has_fresh_external_detection(
                 state.vision,
                 hardware.brain.timer(vex::timeUnits::msec))) {
    clear_yolo_detection(hardware, state);
  }

  if (!any_update) {
    state.vision.external_link = hardware.external_vision.link_state();
    state.vision.last_blob_detection.sensor_installed = state.vision.external_link.online;
  }
}

}  // namespace basic::hardware::football_robot_plus::sensors
