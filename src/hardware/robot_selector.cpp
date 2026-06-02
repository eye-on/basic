#include "hardware/robot_selector.h"

namespace basic::hardware::basic_robot {
basic::app::Robot& get_robot();
}

namespace basic::hardware::second_robot {
basic::app::Robot& get_robot();
}

namespace basic::hardware::new_robot {
basic::app::Robot& get_robot();
}

namespace basic::hardware {

basic::app::Robot& get_current_robot() {
  switch (kSelectedRobot) {
    case RobotIdentity::kSecondRobot:
      return second_robot::get_robot();
    case RobotIdentity::kBasicRobot:
      return basic_robot::get_robot();
    case RobotIdentity::kNewRobot:
      return new_robot::get_robot();
    default:
      return basic_robot::get_robot();
  }
}

}  // namespace basic::hardware
