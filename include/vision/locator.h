#ifndef BASIC_INCLUDE_VISION_LOCATOR_H_
#define BASIC_INCLUDE_VISION_LOCATOR_H_

#include "vision/types.h"

namespace basic::vision {

class MonocularLocator {
 public:
  MonocularLocator();
  explicit MonocularLocator(const EstimatorConfig& config);

  void set_config(const EstimatorConfig& config);
  const EstimatorConfig& config() const { return config_; }

  EstimateResult estimate(const CameraModel& camera,
                          const Observation2D& observation,
                          const MetricDimensions& dimensions = {}) const;

  EstimateResult estimate_bearing(const CameraModel& camera,
                                  const Observation2D& observation) const;

  EstimateResult estimate_from_bbox(const CameraModel& camera,
                                    const Observation2D& observation,
                                    const MetricDimensions& dimensions) const;

  Vec2 undistort_pixel(const CameraModel& camera,
                       const Vec2& distorted_pixel) const;

  Vec3 pixel_to_ray(const CameraModel& camera, const Vec2& pixel) const;

 private:
  static double clamp(double value, double lo, double hi);
  static double length(const Vec3& value);
  static Vec3 normalize(const Vec3& value);
  static bool score_is_acceptable(const Observation2D& observation,
                                  const EstimatorConfig& config);
  static bool resolve_center(const Observation2D& observation, Vec2* center_px);
  static double edge_length(const Vec2& a, const Vec2& b);
  static Vec2 centroid(const std::vector<Vec2>& points);

  double estimate_confidence(const Observation2D& observation,
                             const BoundingBox* bbox,
                             bool used_width,
                             bool used_height) const;

  EstimatorConfig config_{};
};

}  // namespace basic::vision

#endif
