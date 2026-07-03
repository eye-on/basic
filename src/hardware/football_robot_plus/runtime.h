#ifndef BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_RUNTIME_H_
#define BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_RUNTIME_H_

#include "vision/locator.h"

namespace basic::hardware::football_robot_plus {

enum class AutoMode {
  kManual,
  kFaceTarget,
  kIntercept,
};

struct InterceptState {
  double gimbal_command_pct{0.0};
  double gimbal_zero_deg{0.0};
  double chassis_strafe_pct{0.0};
};

struct RuntimeState {
  basic::vision::MonocularLocator locator{};
  AutoMode auto_mode{AutoMode::kManual};
  InterceptState intercept{};
  bool intercept_debug_print_enabled{false};
};

}  // namespace basic::hardware::football_robot_plus

#endif  // BASIC_SRC_HARDWARE_FOOTBALL_ROBOT_PLUS_RUNTIME_H_
