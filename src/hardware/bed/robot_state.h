#ifndef BASIC_SRC_HARDWARE_BED_ROBOT_STATE_H_
#define BASIC_SRC_HARDWARE_BED_ROBOT_STATE_H_

#include "hardware/shared/state_types.h"
#include "chassis/yaw_hold.h"
#include "mechanism/arm_2dof.h"
#include "mechanism/dual_pneumatic.h"
#include "mechanism/intake.h"
#include "mechanism/linear_lift.h"
#include "mechanism/pneumatic_gripper.h"

namespace basic::hardware::bed {

struct RobotState {
  basic::hardware::shared::ControllerInputState controller;
  basic::chassis::YawHoldState yaw_hold;
  basic::mechanism::IntakeState intake;
  basic::mechanism::PneumaticGripperState pneumatic_gripper;
  basic::mechanism::Arm2DofState arm_2dof;
  basic::mechanism::LinearLiftState lift;
  basic::mechanism::DualPneumaticState dual_pneumatic;
};

}  // namespace basic::hardware::bed

#endif  // BASIC_SRC_HARDWARE_BED_ROBOT_STATE_H_
