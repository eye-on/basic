#include "vision/locator.h"

#include <algorithm>
#include <cmath>

namespace basic::vision {

namespace {

EstimateResult make_invalid_result(EstimateStatus status) {
  EstimateResult result;
  result.status = status;
  return result;
}

Vec2 normalize_pixel(const CameraModel& camera, const Vec2& pixel) {
  return Vec2{(pixel.x - camera.cx) / camera.fx, (pixel.y - camera.cy) / camera.fy};
}

double select_depth(double depth_from_width, double depth_from_height,
                    double width_pixels, double height_pixels,
                    RangeAxisPreference preference, bool* used_width,
                    bool* used_height) {
  const bool width_ok = is_finite(depth_from_width) && depth_from_width > 0.0;
  const bool height_ok = is_finite(depth_from_height) && depth_from_height > 0.0;

  *used_width = false;
  *used_height = false;

  if (!width_ok && !height_ok) {
    return nan_value();
  }
  if (width_ok && !height_ok) {
    *used_width = true;
    return depth_from_width;
  }
  if (!width_ok && height_ok) {
    *used_height = true;
    return depth_from_height;
  }

  switch (preference) {
    case RangeAxisPreference::kPreferWidth:
      *used_width = true;
      return depth_from_width;
    case RangeAxisPreference::kPreferHeight:
      *used_height = true;
      return depth_from_height;
    case RangeAxisPreference::kMinIfAvailable:
      if (depth_from_width <= depth_from_height) {
        *used_width = true;
        return depth_from_width;
      }
      *used_height = true;
      return depth_from_height;
    case RangeAxisPreference::kMaxIfAvailable:
      if (depth_from_width >= depth_from_height) {
        *used_width = true;
        return depth_from_width;
      }
      *used_height = true;
      return depth_from_height;
    case RangeAxisPreference::kAverageIfAvailable:
    default:
      *used_width = true;
      *used_height = true;
      if (width_pixels > 0.0 && height_pixels > 0.0) {
        return (depth_from_width * width_pixels +
                depth_from_height * height_pixels) /
               (width_pixels + height_pixels);
      }
      return 0.5 * (depth_from_width + depth_from_height);
  }
}

}  // namespace

MonocularLocator::MonocularLocator() { set_config(EstimatorConfig{}); }

MonocularLocator::MonocularLocator(const EstimatorConfig& config) {
  set_config(config);
}

void MonocularLocator::set_config(const EstimatorConfig& config) {
  config_ = config;
  config_.min_detection_score = std::max(config.min_detection_score, 0.0);
  config_.min_bbox_pixels = std::max(config.min_bbox_pixels, 1.0);
  config_.undistort_iterations = std::max(config.undistort_iterations, 1);
}

EstimateResult MonocularLocator::estimate(const CameraModel& camera,
                                          const Observation2D& observation,
                                          const MetricDimensions& dimensions) const {
  if (observation.kind == ObservationKind::kBoundingBox &&
      observation.bbox_px.valid() && dimensions.has_any_linear_dimension()) {
    return estimate_from_bbox(camera, observation, dimensions);
  }

  return estimate_bearing(camera, observation);
}

EstimateResult MonocularLocator::estimate_bearing(
    const CameraModel& camera, const Observation2D& observation) const {
  if (!camera.valid()) {
    return make_invalid_result(EstimateStatus::kInvalidCamera);
  }
  if (!score_is_acceptable(observation, config_)) {
    return make_invalid_result(EstimateStatus::kLowScore);
  }

  Vec2 center_px{};
  if (!resolve_center(observation, &center_px)) {
    return make_invalid_result(EstimateStatus::kMissingCenter);
  }

  const Vec3 ray = pixel_to_ray(camera, center_px);
  if (!is_finite(ray.x) || !is_finite(ray.y) || !is_finite(ray.z)) {
    return make_invalid_result(EstimateStatus::kDegenerateRay);
  }

  EstimateResult result;
  result.kind = EstimateKind::kBearingOnly;
  result.status = EstimateStatus::kOk;
  result.valid = true;
  result.ray_camera = ray;
  result.confidence = estimate_confidence(
      observation,
      observation.bbox_px.valid() ? &observation.bbox_px : nullptr, false, false);
  return result;
}

EstimateResult MonocularLocator::estimate_from_bbox(
    const CameraModel& camera, const Observation2D& observation,
    const MetricDimensions& dimensions) const {
  if (!camera.valid()) {
    return make_invalid_result(EstimateStatus::kInvalidCamera);
  }
  if (!score_is_acceptable(observation, config_)) {
    return make_invalid_result(EstimateStatus::kLowScore);
  }
  if (!observation.bbox_px.valid() ||
      observation.bbox_px.width < config_.min_bbox_pixels ||
      observation.bbox_px.height < config_.min_bbox_pixels) {
    return make_invalid_result(EstimateStatus::kBBoxTooSmall);
  }
  if (!dimensions.has_any_linear_dimension()) {
    return make_invalid_result(EstimateStatus::kMissingDimensions);
  }

  const BoundingBox& bbox = observation.bbox_px;

  Vec2 tl{bbox.x, bbox.y};
  Vec2 tr{bbox.x + bbox.width, bbox.y};
  Vec2 bl{bbox.x, bbox.y + bbox.height};
  Vec2 br{bbox.x + bbox.width, bbox.y + bbox.height};

  if (config_.undistort_input && camera.has_distortion()) {
    tl = undistort_pixel(camera, tl);
    tr = undistort_pixel(camera, tr);
    bl = undistort_pixel(camera, bl);
    br = undistort_pixel(camera, br);
  }

  const double width_pixels =
      0.5 * (edge_length(tl, tr) + edge_length(bl, br));
  const double height_pixels =
      0.5 * (edge_length(tl, bl) + edge_length(tr, br));

  if (!is_finite(width_pixels) || !is_finite(height_pixels) ||
      width_pixels < config_.min_bbox_pixels ||
      height_pixels < config_.min_bbox_pixels) {
    return make_invalid_result(EstimateStatus::kBBoxTooSmall);
  }

  const Vec2 center_px{
      (tl.x + tr.x + bl.x + br.x) * 0.25,
      (tl.y + tr.y + bl.y + br.y) * 0.25,
  };
  const Vec2 center_normalized = normalize_pixel(camera, center_px);
  const Vec3 ray = normalize(Vec3{center_normalized.x, center_normalized.y, 1.0});

  if (!is_finite(ray.x) || !is_finite(ray.y) || !is_finite(ray.z)) {
    return make_invalid_result(EstimateStatus::kDegenerateRay);
  }

  double depth_from_width = nan_value();
  if (dimensions.has_width && dimensions.width_mm > 0.0 &&
      width_pixels >= config_.min_bbox_pixels) {
    depth_from_width = camera.fx * dimensions.width_mm / width_pixels;
  }

  double depth_from_height = nan_value();
  if (dimensions.has_height && dimensions.height_mm > 0.0 &&
      height_pixels >= config_.min_bbox_pixels) {
    depth_from_height = camera.fy * dimensions.height_mm / height_pixels;
  }

  bool used_width = false;
  bool used_height = false;
  const double depth_mm =
      select_depth(depth_from_width, depth_from_height, width_pixels,
                   height_pixels, config_.range_axis_preference, &used_width,
                   &used_height);

  if (!is_finite(depth_mm) || depth_mm <= 0.0) {
    return make_invalid_result(EstimateStatus::kMissingDimensions);
  }

  EstimateResult result;
  result.kind = EstimateKind::kApproximatePosition;
  result.status = EstimateStatus::kOk;
  result.valid = true;
  result.ray_camera = ray;
  result.position_camera_mm = Vec3{
      center_normalized.x * depth_mm,
      center_normalized.y * depth_mm,
      depth_mm,
  };
  result.depth_mm = depth_mm;
  result.range_mm = length(result.position_camera_mm);
  result.confidence =
      estimate_confidence(observation, &bbox, used_width, used_height);
  result.used_pixel_width = used_width ? width_pixels : 0.0;
  result.used_pixel_height = used_height ? height_pixels : 0.0;
  result.used_width = used_width;
  result.used_height = used_height;
  return result;
}

Vec2 MonocularLocator::undistort_pixel(const CameraModel& camera,
                                       const Vec2& distorted_pixel) const {
  if (!camera.valid() || !is_finite(distorted_pixel.x) ||
      !is_finite(distorted_pixel.y)) {
    return Vec2{nan_value(), nan_value()};
  }
  if (!config_.undistort_input || !camera.has_distortion()) {
    return distorted_pixel;
  }

  const Vec2 distorted_normalized = normalize_pixel(camera, distorted_pixel);
  double x = distorted_normalized.x;
  double y = distorted_normalized.y;

  for (int i = 0; i < config_.undistort_iterations; ++i) {
    const double r2 = x * x + y * y;
    const double radial = 1.0 + camera.k1 * r2 + camera.k2 * r2 * r2 +
                          camera.k3 * r2 * r2 * r2;
    if (!is_finite(radial) || std::abs(radial) < 1e-9) {
      return Vec2{nan_value(), nan_value()};
    }

    const double delta_x =
        2.0 * camera.p1 * x * y + camera.p2 * (r2 + 2.0 * x * x);
    const double delta_y =
        camera.p1 * (r2 + 2.0 * y * y) + 2.0 * camera.p2 * x * y;

    x = (distorted_normalized.x - delta_x) / radial;
    y = (distorted_normalized.y - delta_y) / radial;
  }

  return Vec2{x * camera.fx + camera.cx, y * camera.fy + camera.cy};
}

Vec3 MonocularLocator::pixel_to_ray(const CameraModel& camera,
                                    const Vec2& pixel) const {
  if (!camera.valid()) {
    return Vec3{nan_value(), nan_value(), nan_value()};
  }

  Vec2 corrected = pixel;
  if (config_.undistort_input && camera.has_distortion()) {
    corrected = undistort_pixel(camera, pixel);
  }
  if (!is_finite(corrected.x) || !is_finite(corrected.y)) {
    return Vec3{nan_value(), nan_value(), nan_value()};
  }

  const Vec2 normalized_pixel = normalize_pixel(camera, corrected);
  return normalize(Vec3{normalized_pixel.x, normalized_pixel.y, 1.0});
}

double MonocularLocator::clamp(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

double MonocularLocator::length(const Vec3& value) {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 MonocularLocator::normalize(const Vec3& value) {
  const double value_length = length(value);
  if (!is_finite(value_length) || value_length <= 1e-9) {
    return Vec3{nan_value(), nan_value(), nan_value()};
  }

  return Vec3{value.x / value_length, value.y / value_length,
              value.z / value_length};
}

bool MonocularLocator::score_is_acceptable(const Observation2D& observation,
                                           const EstimatorConfig& config) {
  if (!is_finite(observation.score)) {
    return true;
  }
  return observation.score >= config.min_detection_score;
}

bool MonocularLocator::resolve_center(const Observation2D& observation,
                                      Vec2* center_px) {
  switch (observation.kind) {
    case ObservationKind::kCenter:
      if (is_finite(observation.center_px.x) &&
          is_finite(observation.center_px.y)) {
        *center_px = observation.center_px;
        return true;
      }
      break;
    case ObservationKind::kBoundingBox:
      if (observation.bbox_px.valid()) {
        *center_px = observation.bbox_px.center();
        return true;
      }
      break;
    case ObservationKind::kKeypoints:
      if (!observation.keypoints_px.empty()) {
        *center_px = centroid(observation.keypoints_px);
        return is_finite(center_px->x) && is_finite(center_px->y);
      }
      break;
  }

  if (observation.bbox_px.valid()) {
    *center_px = observation.bbox_px.center();
    return true;
  }
  if (!observation.keypoints_px.empty()) {
    *center_px = centroid(observation.keypoints_px);
    return is_finite(center_px->x) && is_finite(center_px->y);
  }
  if (is_finite(observation.center_px.x) && is_finite(observation.center_px.y)) {
    *center_px = observation.center_px;
    return true;
  }
  return false;
}

double MonocularLocator::edge_length(const Vec2& a, const Vec2& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

Vec2 MonocularLocator::centroid(const std::vector<Vec2>& points) {
  if (points.empty()) {
    return Vec2{nan_value(), nan_value()};
  }

  double sum_x = 0.0;
  double sum_y = 0.0;
  int count = 0;
  for (const Vec2& point : points) {
    if (!is_finite(point.x) || !is_finite(point.y)) {
      continue;
    }
    sum_x += point.x;
    sum_y += point.y;
    ++count;
  }

  if (count <= 0) {
    return Vec2{nan_value(), nan_value()};
  }

  return Vec2{sum_x / count, sum_y / count};
}

double MonocularLocator::estimate_confidence(const Observation2D& observation,
                                             const BoundingBox* bbox,
                                             bool used_width,
                                             bool used_height) const {
  const double score_factor =
      is_finite(observation.score) ? clamp(observation.score, 0.0, 1.0) : 1.0;

  double geometry_factor = 1.0;
  if (bbox != nullptr && bbox->valid()) {
    const double longest_edge = std::max(bbox->width, bbox->height);
    geometry_factor =
        clamp(longest_edge / (config_.min_bbox_pixels * 8.0), 0.25, 1.0);
  }

  double fusion_factor = 0.85;
  if (used_width && used_height) {
    fusion_factor = 1.0;
  } else if (used_width || used_height) {
    fusion_factor = 0.9;
  }

  return clamp(score_factor * geometry_factor * fusion_factor, 0.0, 1.0);
}

}  // namespace basic::vision
