#ifndef BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_VISION_H_
#define BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_VISION_H_

#include "mechanism/pneumatic_motor_actuator.h"
#include "vision/locator.h"

namespace basic::hardware::football_robot {

inline constexpr double kDefaultFootballDiameterMm = 220.0;

struct YoloBoundingBoxPx {
  double x{0.0};
  double y{0.0};
  double width{0.0};
  double height{0.0};

  bool valid() const {
    return basic::vision::is_finite(x) && basic::vision::is_finite(y) &&
           basic::vision::is_finite(width) && basic::vision::is_finite(height) &&
           width > 0.0 && height > 0.0;
  }

  basic::vision::BoundingBox to_vision_bbox() const {
    return basic::vision::BoundingBox{x, y, width, height};
  }
};

struct YoloDetection {
  bool has_detection{false};
  int class_id{-1};
  double score{1.0};
  double image_width_px{800.0};
  double image_height_px{800.0};
  YoloBoundingBoxPx bbox_px{};
};

struct FootballVisionConfig {
  // Intrinsics are interpreted in this reference image size and will be
  // scaled to the YOLO image size on submission when needed.
  double image_width_px{800.0};
  double image_height_px{800.0};
  double football_diameter_mm{kDefaultFootballDiameterMm};
  int expected_class_id{-1};
  basic::vision::CameraModel camera{
      0.0,
      0.0,
      basic::vision::nan_value(),
      basic::vision::nan_value(),
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
  };
  basic::vision::EstimatorConfig estimator{};
};

struct FootballVisionState {
  FootballVisionConfig config{};
  YoloDetection last_detection{};
  // Camera frame follows the pinhole convention used by the solver:
  // +x right in the image, +y down in the image, +z forward from the lens.
  basic::vision::EstimateResult last_estimate{};
  bool estimate_available{false};
  bool class_filter_passed{true};
  int last_update_time_ms{0};
};

basic::vision::CameraModel resolve_camera_model(const FootballVisionConfig& config);
basic::vision::CameraModel resolve_camera_model(
    const FootballVisionConfig& config,
    const YoloDetection& detection);

basic::vision::MetricDimensions football_metric_dimensions(double football_diameter_mm);

basic::vision::EstimateResult estimate_football_from_yolo(
    basic::vision::MonocularLocator& locator,
    const FootballVisionConfig& config,
    const YoloDetection& detection);

basic::vision::EstimateResult estimate_football_from_yolo(
    const FootballVisionConfig& config,
    const YoloDetection& detection);

void configure_vision(const FootballVisionConfig& config);
basic::vision::EstimateResult submit_yolo_detection(const YoloDetection& detection);
void clear_yolo_detection();
FootballVisionState get_vision_state();
basic::mechanism::PneumaticMotorActuatorState get_actuator_state();
basic::mechanism::PneumaticMotorActuatorState refresh_actuator_state();
void set_actuator_motor_angle_a();
void set_actuator_motor_angle_b();
void toggle_actuator_motor_angle_state();
void update_actuator_motor_to_target();

}  // namespace basic::hardware::football_robot

#endif  // BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_VISION_H_
