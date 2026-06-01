#ifndef BASIC_INCLUDE_DEVICE_CONFIG_H_
#define BASIC_INCLUDE_DEVICE_CONFIG_H_

#include "vex.h"

namespace basic::device {

struct MotorConfig {
  int port{0};
  vex::gearSetting gear_ratio{vex::ratio6_1};
  bool reversed{false};
};

struct DigitalOutConfig {
  vex::triport::port& port;
};

}  // namespace basic::device

#endif
