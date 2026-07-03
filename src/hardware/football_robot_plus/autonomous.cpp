#include "chassis/x_chassis.h"
#include "chassis/x_drive.h"
#include "hardware/football_robot_plus/robot_hardware.h"
#include "hardware/football_robot_plus/robot_state.h"
#include "hardware/football_robot_plus/runtime.h"
#include "hardware/football_robot_plus/sensors.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace basic::hardware::football_robot_plus::autonomous {

namespace {

constexpr int kVisionStaleTimeoutMs = 500;
constexpr double kPi = 3.14159265358979323846;
constexpr double kAutoHeadingToleranceRad = 0.09;
constexpr double kAutoPickupHeadingToleranceRad = 0.05;
constexpr double kAutoTurnGainPctPerRad = 180.0;
constexpr double kAutoMinTurnPct = 8.0;
constexpr double kAutoMaxTurnPct = 20.0;
constexpr double kAutoTurnDirectionSign = 1.0;
constexpr double kGimbalPositionLimitDeg = 45.0;
constexpr double kGimbalTrackToleranceDeg = 2.0;
constexpr double kGimbalTrackGainPctPerDeg = 0.15;
constexpr double kGimbalTrackMaxPct = 25.0;
constexpr double kGimbalTrackDirectionSign = -1.0;
constexpr double kGimbalTrackRampPctPerStep = 4.0;
constexpr double kInterceptStrafeGainPctPerDeg = 1.2;
constexpr double kInterceptStrafeMaxPct = 35.0;
constexpr double kInterceptStrafeDeadbandDeg = 3.0;
constexpr double kInterceptStrafeRampPctPerStep = 6.0;
constexpr double kInterceptStrafeDirectionSign = -1.0;
constexpr double kFallbackImageHalfFovDeg = 30.0;

double clamp_value(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

double clamp_abs(double value, double max_abs) {
  return clamp_value(value, -max_abs, max_abs);
}

double ramp_toward(double current, double target, double max_step) {
  if (max_step <= 0.0) {
    return target;
  }
  if (target > current) {
    return std::min(current + max_step, target);
  }
  return std::max(current - max_step, target);
}

bool is_positive_finite(double value) {
  return basic::vision::is_finite(value) && value > 0.0;
}

bool is_finite(double value) {
  return basic::vision::is_finite(value);
}

double radians_to_degrees(double radians) {
  return radians * 180.0 / kPi;
}

void stop_drive(RobotHardware& hardware, vex::brakeType brake_type) {
  basic::chassis::x_chassis_stop(hardware.football_chassis, brake_type);
}

void apply_drive_request(
    RobotHardware& hardware,
    double forward_pct,
    double strafe_pct,
    double turn_pct,
    vex::brakeType brake_type) {
  double fl_pct = forward_pct + strafe_pct + turn_pct;
  double fr_pct = forward_pct - strafe_pct - turn_pct;
  double bl_pct = forward_pct - strafe_pct + turn_pct;
  double br_pct = forward_pct + strafe_pct - turn_pct;

  const double max_pct = std::max(
      {std::fabs(fl_pct), std::fabs(fr_pct), std::fabs(bl_pct), std::fabs(br_pct)});
  if (max_pct > 100.0) {
    const double scale = 100.0 / max_pct;
    fl_pct *= scale;
    fr_pct *= scale;
    bl_pct *= scale;
    br_pct *= scale;
  }

  basic::chassis::x_drive_set_output(
      hardware.football_chassis,
      fl_pct,
      fr_pct,
      bl_pct,
      br_pct,
      brake_type);
}

void reset_intercept_state(
    RobotHardware& hardware,
    RobotState& state,
    RuntimeState& runtime) {
  sensors::refresh_camera_gimbal_state(hardware, state);
  runtime.intercept = InterceptState{};
  runtime.intercept.gimbal_zero_deg = state.camera_gimbal.motor_position_deg;
}

void handle_auto_mode_toggle(
    RobotHardware& hardware,
    RobotState& state,
    RuntimeState& runtime) {
  if (!state.controller.press_y) {
    return;
  }

  runtime.auto_mode =
      runtime.auto_mode == AutoMode::kFaceTarget ? AutoMode::kManual : AutoMode::kFaceTarget;
  runtime.intercept_debug_print_enabled = runtime.auto_mode == AutoMode::kIntercept;
  stop_drive(hardware, runtime.auto_mode == AutoMode::kManual ? vex::coast : vex::hold);
  if (runtime.auto_mode != AutoMode::kManual) {
    reset_intercept_state(hardware, state, runtime);
  }
}

void handle_intercept_mode_toggle(
    RobotHardware& hardware,
    RobotState& state,
    RuntimeState& runtime) {
  if (!state.controller.press_b) {
    return;
  }

  runtime.auto_mode =
      runtime.auto_mode == AutoMode::kIntercept ? AutoMode::kManual : AutoMode::kIntercept;
  runtime.intercept_debug_print_enabled = runtime.auto_mode == AutoMode::kIntercept;
  stop_drive(hardware, runtime.auto_mode == AutoMode::kManual ? vex::coast : vex::hold);
  if (runtime.auto_mode == AutoMode::kIntercept) {
    reset_intercept_state(hardware, state, runtime);
  }
}

bool has_recent_target(const FootballVisionState& vision, int now_ms) {
  if (!vision.last_detection.has_detection || !vision.class_filter_passed) {
    return false;
  }

  if (now_ms < vision.last_update_time_ms ||
      now_ms - vision.last_update_time_ms > kVisionStaleTimeoutMs) {
    return false;
  }

  return vision.last_detection.bbox_px.valid() ||
         (vision.estimate_available && vision.last_estimate.valid);
}

double resolve_image_width_px(const FootballVisionState& vision) {
  if (is_positive_finite(vision.last_detection.image_width_px)) {
    return vision.last_detection.image_width_px;
  }
  if (is_positive_finite(vision.config.image_width_px)) {
    return vision.config.image_width_px;
  }
  return 0.0;
}

double resolve_lateral_error_norm(
    const FootballVisionState& vision,
    double image_width_px) {
  if (vision.estimate_available && vision.last_estimate.valid) {
    if (camera_extrinsics_valid(vision.config.camera_extrinsics)) {
      const basic::vision::Vec3 robot_ray = camera_vector_to_robot_frame(
          vision.config.camera_extrinsics,
          vision.last_estimate.ray_camera);
      if (basic::vision::is_finite(robot_ray.x)) {
        return clamp_abs(robot_ray.x, 1.0);
      }
    }

    if (basic::vision::is_finite(vision.last_estimate.ray_camera.x)) {
      return clamp_abs(vision.last_estimate.ray_camera.x, 1.0);
    }
  }

  if (vision.last_detection.bbox_px.valid() && image_width_px > 0.0) {
    const double image_center_px = image_width_px * 0.5;
    const double bbox_center_px =
        vision.last_detection.bbox_px.x + vision.last_detection.bbox_px.width * 0.5;
    return clamp_abs((bbox_center_px - image_center_px) / image_center_px, 1.0);
  }

  return 0.0;
}

double resolve_heading_error_rad(const FootballVisionState& vision) {
  if (vision.estimate_available && vision.last_estimate.valid) {
    const basic::vision::Vec3 robot_position = camera_point_to_robot_frame(
        vision.config.camera_extrinsics,
        vision.last_estimate.position_camera_mm);
    if (is_finite(robot_position.x) && is_finite(robot_position.z) &&
        (std::fabs(robot_position.x) > 1e-6 || std::fabs(robot_position.z) > 1e-6)) {
      return std::atan2(robot_position.x, robot_position.z);
    }

    const basic::vision::Vec3 robot_ray = camera_vector_to_robot_frame(
        vision.config.camera_extrinsics,
        vision.last_estimate.ray_camera);
    if (is_finite(robot_ray.x) && is_finite(robot_ray.z) &&
        (std::fabs(robot_ray.x) > 1e-6 || std::fabs(robot_ray.z) > 1e-6)) {
      return std::atan2(robot_ray.x, robot_ray.z);
    }
  }

  if (vision.last_detection.bbox_px.valid()) {
    const basic::vision::CameraModel camera =
        resolve_camera_model(vision.config, vision.last_detection);
    const double bbox_center_x_px =
        vision.last_detection.bbox_px.x + vision.last_detection.bbox_px.width * 0.5;
    if (camera.valid() && is_finite(bbox_center_x_px)) {
      const basic::vision::Vec3 camera_ray{
          (bbox_center_x_px - camera.cx) / camera.fx,
          0.0,
          1.0,
      };
      const basic::vision::Vec3 robot_ray = camera_vector_to_robot_frame(
          vision.config.camera_extrinsics,
          camera_ray);
      if (is_finite(robot_ray.x) && is_finite(robot_ray.z) &&
          (std::fabs(robot_ray.x) > 1e-6 || std::fabs(robot_ray.z) > 1e-6)) {
        return std::atan2(robot_ray.x, robot_ray.z);
      }
    }
  }

  return basic::vision::nan_value();
}

void run_face_target_step(RobotHardware& hardware, RobotState& state) {
  const int now_ms = state.controller.time_ms > 0
                         ? state.controller.time_ms
                         : hardware.brain.timer(vex::timeUnits::msec);
  const FootballVisionState vision = state.vision;
  if (!has_recent_target(vision, now_ms)) {
    stop_drive(hardware, vex::hold);
    return;
  }

  const double image_width_px = resolve_image_width_px(vision);
  const double lateral_error_norm = resolve_lateral_error_norm(vision, image_width_px);
  const double heading_error_rad = resolve_heading_error_rad(vision);
  const bool has_heading_error = is_finite(heading_error_rad);

  if (has_heading_error &&
      std::fabs(heading_error_rad) <= kAutoPickupHeadingToleranceRad) {
    stop_drive(hardware, vex::hold);
    return;
  }

  double turn_pct = 0.0;
  if (has_heading_error) {
    turn_pct = clamp_abs(
        heading_error_rad * kAutoTurnGainPctPerRad * kAutoTurnDirectionSign,
        kAutoMaxTurnPct);
    if (std::fabs(heading_error_rad) > kAutoHeadingToleranceRad &&
        std::fabs(turn_pct) < kAutoMinTurnPct) {
      turn_pct = turn_pct >= 0.0 ? kAutoMinTurnPct : -kAutoMinTurnPct;
    }
  } else {
    turn_pct = clamp_abs(
        lateral_error_norm * (kAutoTurnGainPctPerRad * 0.8) * kAutoTurnDirectionSign,
        kAutoMaxTurnPct);
  }

  apply_drive_request(hardware, 0.0, 0.0, turn_pct, vex::hold);
}

bool has_current_intercept_target_measurement(
    const FootballVisionState& vision,
    int now_ms) {
  return vision.last_detection.has_detection && vision.class_filter_passed &&
         vision.last_detection.bbox_px.valid() &&
         now_ms >= vision.last_update_time_ms &&
         now_ms - vision.last_update_time_ms <= kVisionStaleTimeoutMs;
}

double resolve_target_bearing_deg(const FootballVisionState& vision) {
  if (!vision.last_detection.bbox_px.valid()) {
    return basic::vision::nan_value();
  }

  const double bbox_center_x_px =
      vision.last_detection.bbox_px.x + vision.last_detection.bbox_px.width * 0.5;
  const basic::vision::CameraModel camera =
      resolve_camera_model(vision.config, vision.last_detection);
  if (camera.valid() && is_finite(bbox_center_x_px) && is_finite(camera.cx) &&
      is_finite(camera.fx) && std::fabs(camera.fx) > 1e-6) {
    return radians_to_degrees(std::atan((bbox_center_x_px - camera.cx) / camera.fx));
  }

  const double image_width_px = resolve_image_width_px(vision);
  if (!is_positive_finite(image_width_px) || !is_finite(bbox_center_x_px)) {
    return basic::vision::nan_value();
  }

  const double image_center_px = image_width_px * 0.5;
  return ((bbox_center_x_px - image_center_px) / image_center_px) *
         kFallbackImageHalfFovDeg;
}

double current_gimbal_relative_deg(const RobotState& state, const RuntimeState& runtime) {
  return state.camera_gimbal.motor_position_deg - runtime.intercept.gimbal_zero_deg;
}

void set_intercept_gimbal_output_pct(
    RobotHardware& hardware,
    RobotState& state,
    RuntimeState& runtime,
    double target_pct) {
  const double relative_deg = current_gimbal_relative_deg(state, runtime);
  if (relative_deg >= kGimbalPositionLimitDeg && target_pct > 0.0) {
    target_pct = 0.0;
  } else if (relative_deg <= -kGimbalPositionLimitDeg && target_pct < 0.0) {
    target_pct = 0.0;
  }

  runtime.intercept.gimbal_command_pct = ramp_toward(
      runtime.intercept.gimbal_command_pct,
      clamp_abs(target_pct, 100.0),
      kGimbalTrackRampPctPerStep);
  basic::mechanism::camera_gimbal_set_output(
      hardware.camera_gimbal,
      runtime.intercept.gimbal_command_pct);
  sensors::refresh_camera_gimbal_state(hardware, state);
}

void run_intercept_gimbal_track_step(
    RobotHardware& hardware,
    RobotState& state,
    RuntimeState& runtime,
    const FootballVisionState& vision) {
  const double bearing_deg = resolve_target_bearing_deg(vision);
  if (!is_finite(bearing_deg) || std::fabs(bearing_deg) <= kGimbalTrackToleranceDeg) {
    set_intercept_gimbal_output_pct(hardware, state, runtime, 0.0);
    return;
  }

  const double motor_pct = clamp_abs(
      bearing_deg * kGimbalTrackGainPctPerDeg * kGimbalTrackDirectionSign,
      kGimbalTrackMaxPct);
  set_intercept_gimbal_output_pct(hardware, state, runtime, motor_pct);
}

double make_intercept_strafe_pct_from_gimbal(double gimbal_relative_deg) {
  if (std::fabs(gimbal_relative_deg) <= kInterceptStrafeDeadbandDeg) {
    return 0.0;
  }

  return clamp_abs(
             gimbal_relative_deg * kInterceptStrafeGainPctPerDeg,
             kInterceptStrafeMaxPct) *
         kInterceptStrafeDirectionSign;
}

void print_intercept_debug(
    const RobotState& state,
    const RuntimeState& runtime,
    int now_ms,
    const FootballVisionState& vision) {
  if (!runtime.intercept_debug_print_enabled || runtime.auto_mode != AutoMode::kIntercept) {
    return;
  }

  const bool has_measurement = has_current_intercept_target_measurement(vision, now_ms);
  const double bearing_deg =
      has_measurement ? resolve_target_bearing_deg(vision) : basic::vision::nan_value();
  std::printf(
      "ts=%d seen=%d bearing=%.1f rel=%.1f gp=%.1f sp=%.1f\n",
      now_ms,
      has_measurement ? 1 : 0,
      bearing_deg,
      current_gimbal_relative_deg(state, runtime),
      runtime.intercept.gimbal_command_pct,
      runtime.intercept.chassis_strafe_pct);
}

void run_intercept_step(
    RobotHardware& hardware,
    RobotState& state,
    RuntimeState& runtime) {
  const int now_ms = hardware.brain.timer(vex::timeUnits::msec);
  const FootballVisionState vision = state.vision;
  sensors::refresh_camera_gimbal_state(hardware, state);
  print_intercept_debug(state, runtime, now_ms, vision);

  if (!has_current_intercept_target_measurement(vision, now_ms)) {
    set_intercept_gimbal_output_pct(hardware, state, runtime, 0.0);
    runtime.intercept.chassis_strafe_pct = ramp_toward(
        runtime.intercept.chassis_strafe_pct,
        0.0,
        kInterceptStrafeRampPctPerStep);
    apply_drive_request(
        hardware,
        0.0,
        runtime.intercept.chassis_strafe_pct,
        0.0,
        vex::hold);
    return;
  }

  run_intercept_gimbal_track_step(hardware, state, runtime, vision);
  runtime.intercept.chassis_strafe_pct = ramp_toward(
      runtime.intercept.chassis_strafe_pct,
      make_intercept_strafe_pct_from_gimbal(current_gimbal_relative_deg(state, runtime)),
      kInterceptStrafeRampPctPerStep);
  apply_drive_request(
      hardware,
      0.0,
      runtime.intercept.chassis_strafe_pct,
      0.0,
      vex::hold);
}

}  // namespace

void step(RobotHardware& hardware, RobotState& state, RuntimeState& runtime) {
  handle_auto_mode_toggle(hardware, state, runtime);
  handle_intercept_mode_toggle(hardware, state, runtime);

  if (runtime.auto_mode == AutoMode::kFaceTarget) {
    run_face_target_step(hardware, state);
    return;
  }

  if (runtime.auto_mode == AutoMode::kIntercept) {
    run_intercept_step(hardware, state, runtime);
  }
}

}  // namespace basic::hardware::football_robot_plus::autonomous
