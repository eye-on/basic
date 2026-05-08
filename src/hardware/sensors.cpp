#include "hardware/sensors.h"
#include "hardware/robots/robot_state.h"

#include "v5_apiuser.h"

namespace basic::hardware::robots {

namespace {
  
}

void sensor_update(RobotHardware& hardware, RobotState& state,
                   const vex::color target,
                   const int wait_frame, const int continuous_frame) {
  IndexedMechanismMode& indexed_mode = state.mechanism.indexed_mode;
  static int current_frame = 0;
  ++current_frame;
  static int start_wait_frame = -1;
  static int start_sort_frame = -1;
  if (indexed_mode != IndexedMechanismMode::kLegacyIntake &&
    indexed_mode != IndexedMechanismMode::kSortIntake) {
    start_wait_frame = -1;
    start_sort_frame = -1;
    return;
  }

  if (indexed_mode == IndexedMechanismMode::kLegacyIntake) {
    if (hardware.color_sensor.color() == target) {
      if (start_wait_frame == -1) {
        start_wait_frame = current_frame;
      }
      if (current_frame - start_wait_frame >= wait_frame) {
        indexed_mode = IndexedMechanismMode::kSortIntake;
        start_sort_frame = current_frame;
        start_wait_frame = -1; 
      }
    } else {
      start_wait_frame = -1;
    }
  }
  else if (indexed_mode == IndexedMechanismMode::kSortIntake) {
    if (start_sort_frame == -1) {
      start_sort_frame = current_frame;
    }
    if (current_frame - start_sort_frame >= continuous_frame) {
      indexed_mode = IndexedMechanismMode::kLegacyIntake;
      start_sort_frame = -1;
      start_wait_frame = -1;
    }
  }
}

}// namespace basic::hardware::robots
