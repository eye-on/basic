#ifndef BASIC_INCLUDE_IDENTIFY_VISION_SENSOR_H_
#define BASIC_INCLUDE_IDENTIFY_VISION_SENSOR_H_

#include <cstdint>

#include "vex.h"

namespace basic::identify {

inline constexpr int kVisionSensorImageWidthPx = 316;
inline constexpr int kVisionSensorImageHeightPx = 212;
inline constexpr uint8_t kDefaultVisionSensorBrightness = 50;

enum class VisionTargetColor {
  kRed = 0,
  kYellowGreen = 1,
  kPurple = 2,
};

struct LargestBlobDetection {
  bool sensor_installed{false};
  bool has_detection{false};
  VisionTargetColor color{VisionTargetColor::kRed};
  int object_count{0};
  int signature_id{0};
  int origin_x_px{0};
  int origin_y_px{0};
  int center_x_px{0};
  int center_y_px{0};
  int width_px{0};
  int height_px{0};
  double angle_deg{0.0};
  double image_width_px{kVisionSensorImageWidthPx};
  double image_height_px{kVisionSensorImageHeightPx};

  bool valid() const {
    return sensor_installed && has_detection && width_px > 0 && height_px > 0;
  }
};

class VisionSensorIdentifier final {
 public:
  explicit VisionSensorIdentifier(
      int32_t port,
      uint8_t brightness = kDefaultVisionSensorBrightness);

  void set_target_color(VisionTargetColor color);
  VisionTargetColor target_color() const;

  LargestBlobDetection refresh();
  LargestBlobDetection detect_largest_blob(VisionTargetColor color);
  const LargestBlobDetection& last_detection() const;

  bool installed();
  vex::vision& sensor();

 private:
  static vex::vision::signature make_red_signature();
  static vex::vision::signature make_yellow_green_signature();
  static vex::vision::signature make_purple_signature();

  vex::vision::signature& signature_for(VisionTargetColor color);
  LargestBlobDetection snapshot_from_sensor(VisionTargetColor color);

  vex::vision::signature red_signature_;
  vex::vision::signature yellow_green_signature_;
  vex::vision::signature purple_signature_;
  vex::vision sensor_;
  VisionTargetColor target_color_{VisionTargetColor::kRed};
  LargestBlobDetection last_detection_{};
};

char color_code(VisionTargetColor color);

}  // namespace basic::identify

#endif  // BASIC_INCLUDE_IDENTIFY_VISION_SENSOR_H_
