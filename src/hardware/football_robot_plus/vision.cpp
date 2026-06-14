#include "hardware/football_robot_plus/vision.h"

namespace basic::hardware::football_robot_plus {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool is_positive_finite(double value) {
  return basic::vision::is_finite(value) && value > 0.0;
}

double degrees_to_radians(double degrees) {
  return degrees * kPi / 180.0;
}

basic::vision::Vec3 rotate_x(const basic::vision::Vec3& value, double radians) {
  const double c = std::cos(radians);
  const double s = std::sin(radians);
  return basic::vision::Vec3{
      value.x,
      c * value.y - s * value.z,
      s * value.y + c * value.z,
  };
}

basic::vision::Vec3 rotate_y(const basic::vision::Vec3& value, double radians) {
  const double c = std::cos(radians);
  const double s = std::sin(radians);
  return basic::vision::Vec3{
      c * value.x + s * value.z,
      value.y,
      -s * value.x + c * value.z,
  };
}

basic::vision::Vec3 rotate_z(const basic::vision::Vec3& value, double radians) {
  const double c = std::cos(radians);
  const double s = std::sin(radians);
  return basic::vision::Vec3{
      c * value.x - s * value.y,
      s * value.x + c * value.y,
      value.z,
  };
}

void scale_intrinsics(
    basic::vision::CameraModel& camera,
    double scale_x,
    double scale_y) {
  camera.fx *= scale_x;
  camera.fy *= scale_y;
  if (basic::vision::is_finite(camera.cx)) {
    camera.cx *= scale_x;
  }
  if (basic::vision::is_finite(camera.cy)) {
    camera.cy *= scale_y;
  }
}

double select_image_extent(double detection_extent, double config_extent) {
  if (is_positive_finite(detection_extent)) {
    return detection_extent;
  }
  return config_extent;
}

double resolve_principal_point(double configured_value, double image_extent) {
  if (basic::vision::is_finite(configured_value)) {
    return configured_value;
  }
  if (is_positive_finite(image_extent)) {
    return image_extent * 0.5;
  }
  return basic::vision::nan_value();
}

bool detection_matches_expected_class(
    const FootballVisionConfig& config,
    const YoloDetection& detection) {
  return config.expected_class_id < 0 || detection.class_id == config.expected_class_id;
}

basic::vision::EstimateResult make_invalid_estimate(basic::vision::EstimateStatus status) {
  basic::vision::EstimateResult result;
  result.status = status;
  return result;
}

basic::vision::Observation2D make_observation(const YoloDetection& detection) {
  basic::vision::Observation2D observation;
  observation.kind = basic::vision::ObservationKind::kBoundingBox;
  observation.class_id = detection.class_id;
  observation.score = detection.score;
  observation.bbox_px = detection.bbox_px.to_vision_bbox();
  return observation;
}

}  // namespace

basic::vision::CameraModel resolve_camera_model(const FootballVisionConfig& config) {
  basic::vision::CameraModel camera = config.camera;
  camera.cx = resolve_principal_point(camera.cx, config.image_width_px);
  camera.cy = resolve_principal_point(camera.cy, config.image_height_px);
  return camera;
}

basic::vision::CameraModel resolve_camera_model(
    const FootballVisionConfig& config,
    const YoloDetection& detection) {
  basic::vision::CameraModel camera = config.camera;
  const double image_width_px =
      select_image_extent(detection.image_width_px, config.image_width_px);
  const double image_height_px =
      select_image_extent(detection.image_height_px, config.image_height_px);
  if (is_positive_finite(config.image_width_px) && is_positive_finite(image_width_px) &&
      is_positive_finite(config.image_height_px) && is_positive_finite(image_height_px)) {
    scale_intrinsics(
        camera,
        image_width_px / config.image_width_px,
        image_height_px / config.image_height_px);
  }
  camera.cx = resolve_principal_point(camera.cx, image_width_px);
  camera.cy = resolve_principal_point(camera.cy, image_height_px);
  return camera;
}

basic::vision::MetricDimensions football_metric_dimensions(double football_diameter_mm) {
  basic::vision::MetricDimensions dimensions;
  if (!is_positive_finite(football_diameter_mm)) {
    return dimensions;
  }

  dimensions.has_width = true;
  dimensions.width_mm = football_diameter_mm;
  dimensions.has_height = true;
  dimensions.height_mm = football_diameter_mm;
  return dimensions;
}

basic::vision::EstimateResult estimate_football_from_yolo(
    basic::vision::MonocularLocator& locator,
    const FootballVisionConfig& config,
    const YoloDetection& detection) {
  if (!detection.has_detection) {
    return make_invalid_estimate(basic::vision::EstimateStatus::kUnsupportedObservation);
  }
  if (!detection.bbox_px.valid()) {
    return make_invalid_estimate(basic::vision::EstimateStatus::kBBoxTooSmall);
  }
  if (!detection_matches_expected_class(config, detection)) {
    return make_invalid_estimate(basic::vision::EstimateStatus::kUnsupportedObservation);
  }

  locator.set_config(config.estimator);
  return locator.estimate(
      resolve_camera_model(config, detection),
      make_observation(detection),
      football_metric_dimensions(config.football_diameter_mm));
}

basic::vision::EstimateResult estimate_football_from_yolo(
    const FootballVisionConfig& config,
    const YoloDetection& detection) {
  basic::vision::MonocularLocator locator(config.estimator);
  return estimate_football_from_yolo(locator, config, detection);
}

bool camera_extrinsics_valid(const CameraExtrinsics& extrinsics) {
  return basic::vision::is_finite(extrinsics.x_mm) &&
         basic::vision::is_finite(extrinsics.y_mm) &&
         basic::vision::is_finite(extrinsics.z_mm) &&
         basic::vision::is_finite(extrinsics.roll_deg) &&
         basic::vision::is_finite(extrinsics.pitch_deg) &&
         basic::vision::is_finite(extrinsics.yaw_deg);
}

basic::vision::Vec3 camera_vector_to_robot_frame(
    const CameraExtrinsics& extrinsics,
    const basic::vision::Vec3& camera_vector) {
  basic::vision::Vec3 rotated = camera_vector;
  rotated = rotate_x(rotated, degrees_to_radians(extrinsics.roll_deg));
  rotated = rotate_y(rotated, degrees_to_radians(extrinsics.pitch_deg));
  rotated = rotate_z(rotated, degrees_to_radians(extrinsics.yaw_deg));
  return rotated;
}

basic::vision::Vec3 camera_point_to_robot_frame(
    const CameraExtrinsics& extrinsics,
    const basic::vision::Vec3& camera_point_mm) {
  const basic::vision::Vec3 rotated =
      camera_vector_to_robot_frame(extrinsics, camera_point_mm);
  return basic::vision::Vec3{
      rotated.x + extrinsics.x_mm,
      rotated.y + extrinsics.y_mm,
      rotated.z + extrinsics.z_mm,
  };
}

}  // namespace basic::hardware::football_robot_plus
