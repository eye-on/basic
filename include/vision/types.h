#ifndef BASIC_INCLUDE_VISION_TYPES_H_
#define BASIC_INCLUDE_VISION_TYPES_H_

#include <cmath>
#include <limits>
#include <vector>

namespace basic::vision {

inline double nan_value() {
  return std::numeric_limits<double>::quiet_NaN();
}

inline bool is_finite(double value) {
  return std::isfinite(value);
}

struct Vec2 {
  double x{0.0};
  double y{0.0};
};

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct CameraModel {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};

  double k1{0.0};
  double k2{0.0};
  double p1{0.0};
  double p2{0.0};
  double k3{0.0};

  bool has_distortion() const {
    return k1 != 0.0 || k2 != 0.0 || p1 != 0.0 || p2 != 0.0 || k3 != 0.0;
  }

  bool valid() const {
    return is_finite(fx) && is_finite(fy) && is_finite(cx) && is_finite(cy) &&
           fx > 0.0 && fy > 0.0;
  }
};

struct BoundingBox {
  double x{0.0};
  double y{0.0};
  double width{0.0};
  double height{0.0};

  bool valid() const {
    return is_finite(x) && is_finite(y) && is_finite(width) &&
           is_finite(height) && width > 0.0 && height > 0.0;
  }

  Vec2 center() const {
    return Vec2{x + width * 0.5, y + height * 0.5};
  }
};

enum class ObservationKind {
  kCenter,
  kBoundingBox,
  kKeypoints,
};

struct Observation2D {
  ObservationKind kind{ObservationKind::kCenter};
  int class_id{-1};
  double score{nan_value()};
  Vec2 center_px{};
  BoundingBox bbox_px{};
  std::vector<Vec2> keypoints_px{};
};

struct MetricDimensions {
  bool has_width{false};
  bool has_height{false};
  bool has_depth{false};

  double width_mm{0.0};
  double height_mm{0.0};
  double depth_mm{0.0};

  std::vector<Vec3> model_points_mm{};

  bool has_any_linear_dimension() const {
    return (has_width && width_mm > 0.0) || (has_height && height_mm > 0.0) ||
           (has_depth && depth_mm > 0.0);
  }
};

enum class EstimateKind {
  kInvalid,
  kBearingOnly,
  kApproximatePosition,
  kPose6DReserved,
};

enum class EstimateStatus {
  kOk,
  kInvalidCamera,
  kLowScore,
  kUnsupportedObservation,
  kMissingCenter,
  kMissingDimensions,
  kBBoxTooSmall,
  kDegenerateRay,
};

enum class RangeAxisPreference {
  kAverageIfAvailable,
  kPreferWidth,
  kPreferHeight,
  kMinIfAvailable,
  kMaxIfAvailable,
};

struct EstimatorConfig {
  double min_detection_score{0.0};
  double min_bbox_pixels{4.0};
  RangeAxisPreference range_axis_preference{
      RangeAxisPreference::kAverageIfAvailable};
  bool undistort_input{true};
  int undistort_iterations{8};
};

struct EstimateResult {
  EstimateKind kind{EstimateKind::kInvalid};
  EstimateStatus status{EstimateStatus::kUnsupportedObservation};
  bool valid{false};

  Vec3 ray_camera{0.0, 0.0, 1.0};
  Vec3 position_camera_mm{nan_value(), nan_value(), nan_value()};

  double depth_mm{nan_value()};
  double range_mm{nan_value()};
  double confidence{0.0};

  double used_pixel_width{0.0};
  double used_pixel_height{0.0};
  bool used_width{false};
  bool used_height{false};
};

}  // namespace basic::vision

#endif
