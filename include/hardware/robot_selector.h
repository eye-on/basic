#ifndef BASIC_HARDWARE_ROBOT_SELECTOR_H_
#define BASIC_HARDWARE_ROBOT_SELECTOR_H_

#include "app/robot.h"

namespace basic::hardware {

basic::app::Robot& get_current_robot();

}  // namespace basic::hardware

#endif
