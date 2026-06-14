#ifndef BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_VISION_H_
#define BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_VISION_H_

#include "identify/vision_sensor.h"
#include "vision/locator.h"

namespace basic::hardware::football_robot_plus {

inline constexpr double kDefaultFootballDiameterMm = 127.0;

enum class VisionInputSource {
  kNone,
  kExternalSerial,
};

struct CameraExtrinsics {
  double x_mm{0.0};
  double y_mm{0.0};
  double z_mm{0.0};
  double roll_deg{0.0};
  double pitch_deg{0.0};
  double yaw_deg{0.0};
};

struct ExternalVisionLinkState {
  VisionInputSource source{VisionInputSource::kNone};
  bool online{false};
  int last_rx_time_ms{0};
  int last_source_timestamp_ms{0};
  int parse_error_count{0};
  char reported_color_code{'N'};
};

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
  int source_timestamp_ms{0};
  int class_id{-1};
  double score{1.0};
  double image_width_px{800.0};
  double image_height_px{800.0};
  YoloBoundingBoxPx bbox_px{};
};

struct FootballVisionConfig {
  double image_width_px{800.0};
  double image_height_px{800.0};
  double football_diameter_mm{kDefaultFootballDiameterMm};
  int expected_class_id{-1};
  CameraExtrinsics camera_extrinsics{};
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
  basic::identify::VisionTargetColor target_color{basic::identify::VisionTargetColor::kRed};
  basic::identify::LargestBlobDetection last_blob_detection{};
  YoloDetection last_detection{};
  basic::vision::EstimateResult last_estimate{};
  bool estimate_available{false};
  bool class_filter_passed{true};
  int last_update_time_ms{0};
  int last_detection_rx_time_ms{0};
  ExternalVisionLinkState external_link{};
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

bool camera_extrinsics_valid(const CameraExtrinsics& extrinsics);
basic::vision::Vec3 camera_vector_to_robot_frame(
    const CameraExtrinsics& extrinsics,
    const basic::vision::Vec3& camera_vector);
basic::vision::Vec3 camera_point_to_robot_frame(
    const CameraExtrinsics& extrinsics,
    const basic::vision::Vec3& camera_point_mm);

void configure_vision(const FootballVisionConfig& config);
void set_vision_target_color(basic::identify::VisionTargetColor color);
basic::identify::VisionTargetColor get_vision_target_color();
basic::identify::LargestBlobDetection get_vision_sensor_detection();
basic::vision::EstimateResult submit_yolo_detection(const YoloDetection& detection);
void clear_yolo_detection();
FootballVisionState get_vision_state();

}  // namespace basic::hardware::football_robot_plus

#endif  // BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_VISION_H_
