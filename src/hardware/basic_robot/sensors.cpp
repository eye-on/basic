#include "hardware/basic_robot/sensors.h"

namespace basic::hardware::basic_robot {

void sensor_update(RobotHardware& hardware, RobotState& state, vex::color target) {
  (void)hardware;
  (void)target;
  if (basic::mechanism::indexed_intake_state(hardware.intake).mode ==
      basic::mechanism::IndexedIntakeMode::kLegacyIntake) {
  }
}

}  // namespace basic::hardware::basic_robot
