#include "identify/vision_sensor.h"

namespace basic::identify {

namespace {

struct SignatureSpec {
  int32_t id;
  int32_t u_min;
  int32_t u_max;
  int32_t u_mean;
  int32_t v_min;
  int32_t v_max;
  int32_t v_mean;
  float range;
  int32_t type;
};

constexpr uint32_t kLargestBlobObjectCount = 1;
constexpr int32_t kNormalSignatureType = 0;

// These are starter signatures so the module is wired end-to-end.
// Replace them with values exported from Vision Utility for your sensor,
// target material, and field lighting before match use.
constexpr SignatureSpec kRedSignatureSpec{
    1,
    -76,
    -10,
    -43,
    42,
    127,
    84,
    3.0F,
    kNormalSignatureType,
};

constexpr SignatureSpec kYellowGreenSignatureSpec{
    2,
    -127,
    -42,
    -84,
    -52,
    28,
    -12,
    3.0F,
    kNormalSignatureType,
};

constexpr SignatureSpec kPurpleSignatureSpec{
    3,
    8,
    92,
    50,
    24,
    127,
    75,
    3.0F,
    kNormalSignatureType,
};

vex::vision::signature make_signature(const SignatureSpec& spec) {
  return vex::vision::signature(
      spec.id,
      spec.u_min,
      spec.u_max,
      spec.u_mean,
      spec.v_min,
      spec.v_max,
      spec.v_mean,
      spec.range,
      spec.type);
}

LargestBlobDetection make_detection(
    VisionTargetColor color,
    int signature_id,
    const vex::vision& sensor) {
  LargestBlobDetection detection;
  detection.sensor_installed = true;
  detection.has_detection = sensor.largestObject.exists;
  detection.color = color;
  detection.object_count = sensor.objectCount;
  detection.signature_id = signature_id;
  detection.origin_x_px = sensor.largestObject.originX;
  detection.origin_y_px = sensor.largestObject.originY;
  detection.center_x_px = sensor.largestObject.centerX;
  detection.center_y_px = sensor.largestObject.centerY;
  detection.width_px = sensor.largestObject.width;
  detection.height_px = sensor.largestObject.height;
  detection.angle_deg = sensor.largestObject.angle;
  return detection;
}

}  // namespace

VisionSensorIdentifier::VisionSensorIdentifier(int32_t port, uint8_t brightness)
    : red_signature_(make_red_signature()),
      yellow_green_signature_(make_yellow_green_signature()),
      purple_signature_(make_purple_signature()),
      sensor_(port, brightness, red_signature_, yellow_green_signature_, purple_signature_) {}

void VisionSensorIdentifier::set_target_color(VisionTargetColor color) {
  target_color_ = color;
  last_detection_.color = color;
}

VisionTargetColor VisionSensorIdentifier::target_color() const {
  return target_color_;
}

LargestBlobDetection VisionSensorIdentifier::refresh() {
  last_detection_ = snapshot_from_sensor(target_color_);
  return last_detection_;
}

LargestBlobDetection VisionSensorIdentifier::detect_largest_blob(VisionTargetColor color) {
  set_target_color(color);
  return refresh();
}

const LargestBlobDetection& VisionSensorIdentifier::last_detection() const {
  return last_detection_;
}

bool VisionSensorIdentifier::installed() {
  return sensor_.installed();
}

vex::vision& VisionSensorIdentifier::sensor() {
  return sensor_;
}

vex::vision::signature VisionSensorIdentifier::make_red_signature() {
  return make_signature(kRedSignatureSpec);
}

vex::vision::signature VisionSensorIdentifier::make_yellow_green_signature() {
  return make_signature(kYellowGreenSignatureSpec);
}

vex::vision::signature VisionSensorIdentifier::make_purple_signature() {
  return make_signature(kPurpleSignatureSpec);
}

vex::vision::signature& VisionSensorIdentifier::signature_for(VisionTargetColor color) {
  switch (color) {
    case VisionTargetColor::kRed:
      return red_signature_;
    case VisionTargetColor::kYellowGreen:
      return yellow_green_signature_;
    case VisionTargetColor::kPurple:
      return purple_signature_;
    default:
      return red_signature_;
  }
}

LargestBlobDetection VisionSensorIdentifier::snapshot_from_sensor(VisionTargetColor color) {
  LargestBlobDetection detection;
  detection.color = color;
  detection.signature_id = signature_for(color).id;
  if (!sensor_.installed()) {
    return detection;
  }

  detection.sensor_installed = true;
  vex::vision::signature& signature = signature_for(color);
  detection.object_count = sensor_.takeSnapshot(signature, kLargestBlobObjectCount);
  if (detection.object_count <= 0 || !sensor_.largestObject.exists) {
    return detection;
  }

  return make_detection(color, signature.id, sensor_);
}

char color_code(VisionTargetColor color) {
  switch (color) {
    case VisionTargetColor::kRed:
      return 'R';
    case VisionTargetColor::kYellowGreen:
      return 'Y';
    case VisionTargetColor::kPurple:
      return 'P';
    default:
      return 'N';
  }
}

}  // namespace basic::identify
