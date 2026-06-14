#include "hardware/football_robot/external_vision_serial.h"

#include <cctype>
#include <cstdio>

namespace basic::hardware::football_robot {

namespace {

void reset_packet(ExternalVisionPacket* packet) {
  if (packet == nullptr) {
    return;
  }
  *packet = ExternalVisionPacket{};
}

}  // namespace

ExternalVisionSerial::ExternalVisionSerial(int smart_port_index)
    : port_handle_(smart_port_index, vex::ratio18_1, false) {
  link_state_.source = VisionInputSource::kExternalSerial;
}

void ExternalVisionSerial::initialize() {
  vexGenericSerialEnable(port_handle_.index(), 0);
  vexGenericSerialBaudrate(port_handle_.index(), kExternalVisionBaudrate);
  vexGenericSerialFlush(port_handle_.index());
  line_length_ = 0;
  link_state_ = ExternalVisionLinkState{};
  link_state_.source = VisionInputSource::kExternalSerial;
}

bool ExternalVisionSerial::poll(ExternalVisionPacket* packet) {
  if (packet == nullptr) {
    return false;
  }

  reset_packet(packet);
  bool any_update = false;
  while (vexGenericSerialReceiveAvail(port_handle_.index()) > 0) {
    const int32_t byte = vexGenericSerialReadChar(port_handle_.index());
    if (byte < 0) {
      break;
    }

    const char ch = static_cast<char>(byte);
    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      line_buffer_[line_length_] = '\0';
      trim_trailing_whitespace(line_buffer_);
      if (line_length_ > 0) {
        any_update = handle_line(line_buffer_, packet) || any_update;
      }
      line_length_ = 0;
      continue;
    }

    if (line_length_ < kExternalVisionMaxLineLength - 1) {
      line_buffer_[line_length_++] = ch;
      continue;
    }

    line_length_ = 0;
    ++link_state_.parse_error_count;
  }

  return any_update;
}

bool ExternalVisionSerial::online(int now_ms) const {
  return link_state_.online && now_ms >= link_state_.last_rx_time_ms &&
         now_ms - link_state_.last_rx_time_ms <= kExternalVisionRxTimeoutMs;
}

bool ExternalVisionSerial::handle_line(const char* line, ExternalVisionPacket* packet) {
  const int now_ms = vex::timer::system();
  link_state_.online = true;
  link_state_.last_rx_time_ms = now_ms;

  if (starts_with_prefix(line, "OBS,")) {
    return parse_observation_line(line + 4, packet);
  }

  ++link_state_.parse_error_count;
  return false;
}

bool ExternalVisionSerial::parse_observation_line(
    const char* payload,
    ExternalVisionPacket* packet) {
  int source_timestamp_ms = 0;
  int has_detection = 0;
  char color_code = 'N';
  int class_id = -1;
  double score = 0.0;
  double image_width_px = 0.0;
  double image_height_px = 0.0;
  double x = 0.0;
  double y = 0.0;
  double width = 0.0;
  double height = 0.0;

  if (std::sscanf(
          payload,
          " %d , %d , %c , %d , %lf , %lf , %lf , %lf , %lf , %lf , %lf",
          &source_timestamp_ms,
          &has_detection,
          &color_code,
          &class_id,
          &score,
          &image_width_px,
          &image_height_px,
          &x,
          &y,
          &width,
          &height) != 11) {
    ++link_state_.parse_error_count;
    return false;
  }

  packet->has_observation_update = true;
  packet->detection.has_detection = has_detection != 0;
  packet->detection.source_timestamp_ms = source_timestamp_ms;
  packet->detection.class_id = class_id;
  packet->detection.score = score;
  packet->detection.image_width_px = image_width_px;
  packet->detection.image_height_px = image_height_px;
  packet->detection.bbox_px = YoloBoundingBoxPx{x, y, width, height};
  packet->reported_color_code = color_code;
  packet->target_color = color_from_code(color_code);
  link_state_.last_source_timestamp_ms = source_timestamp_ms;
  link_state_.reported_color_code = color_code;
  return true;
}

bool ExternalVisionSerial::starts_with_prefix(const char* line, const char* prefix) {
  while (*prefix != '\0') {
    if (*line != *prefix) {
      return false;
    }
    ++line;
    ++prefix;
  }
  return true;
}

basic::identify::VisionTargetColor ExternalVisionSerial::color_from_code(char color_code) {
  switch (static_cast<char>(std::toupper(static_cast<unsigned char>(color_code)))) {
    case 'R':
      return basic::identify::VisionTargetColor::kRed;
    case 'Y':
      return basic::identify::VisionTargetColor::kYellowGreen;
    case 'P':
      return basic::identify::VisionTargetColor::kPurple;
    default:
      return basic::identify::VisionTargetColor::kRed;
  }
}

void ExternalVisionSerial::trim_trailing_whitespace(char* text) {
  if (text == nullptr) {
    return;
  }

  int length = 0;
  while (text[length] != '\0') {
    ++length;
  }

  while (length > 0 &&
         std::isspace(static_cast<unsigned char>(text[length - 1])) != 0) {
    text[length - 1] = '\0';
    --length;
  }
}

}  // namespace basic::hardware::football_robot
