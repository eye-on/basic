#ifndef BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_EXTERNAL_VISION_SERIAL_H_
#define BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_EXTERNAL_VISION_SERIAL_H_

#include "hardware/football_robot_plus/vision.h"
#include "vex.h"

namespace basic::hardware::football_robot_plus {

inline constexpr int kExternalVisionBaudrate = 115200;
inline constexpr int kExternalVisionRxTimeoutMs = 250;
inline constexpr int kExternalVisionMaxLineLength = 192;
inline const int kExternalVisionPort = vex::PORT12;

struct ExternalVisionPacket {
  bool has_observation_update{false};
  YoloDetection detection{};
  basic::identify::VisionTargetColor target_color{basic::identify::VisionTargetColor::kRed};
  char reported_color_code{'N'};
};

class ExternalVisionSerial final {
 public:
  explicit ExternalVisionSerial(int smart_port_index = kExternalVisionPort);

  void initialize();
  bool poll(ExternalVisionPacket* packet);

  const ExternalVisionLinkState& link_state() const { return link_state_; }
  bool online(int now_ms) const;

 private:
  bool handle_line(const char* line, ExternalVisionPacket* packet);
  bool parse_observation_line(const char* payload, ExternalVisionPacket* packet);

  static bool starts_with_prefix(const char* line, const char* prefix);
  static basic::identify::VisionTargetColor color_from_code(char color_code);
  static void trim_trailing_whitespace(char* text);

  vex::motor port_handle_;
  ExternalVisionLinkState link_state_{};
  char line_buffer_[kExternalVisionMaxLineLength]{};
  int line_length_{0};
};

}  // namespace basic::hardware::football_robot_plus

#endif  // BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_EXTERNAL_VISION_SERIAL_H_
