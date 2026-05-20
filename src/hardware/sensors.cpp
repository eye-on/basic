#include "hardware/sensors.h"
#include "hardware/robots/robot_state.h"

#include "v5_apiuser.h"

namespace basic::hardware::robots {

namespace {
  const int red_hue = 12;
  const int blue_hue = 213;

  bool hue_within_range(RobotHardware& hardware,int target_hue, int tolerance){
    if(target_hue <= hardware.color_sensor.hue()+tolerance && 
      target_hue >= hardware.color_sensor.hue()-tolerance){
      return true;
    }else{
      return false;
    }
  }
}

void sensor_update(RobotHardware& hardware, RobotState& state,
                   const vex::color target,
                   const int wait_frame, const int continuous_frame) {
  IndexedMechanismMode& indexed_mode = state.mechanism.indexed_mode;
  /*if(indexed_mode==IndexedMechanismMode::kLegacyIntake){
    hardware.controller.Screen.setCursor(1,1);
    hardware.controller.Screen.print("off sorting");
  }
  if(indexed_mode==IndexedMechanismMode::kSortIntake){
    hardware.controller.Screen.setCursor(1,1);
    hardware.controller.Screen.print("on sortting");
  }*/
  static int current_frame = 0;
  ++current_frame;
  static int start_wait_frame = -1;
  static int start_sort_frame = -1;
  static int target_hue = -1;
  if (indexed_mode != IndexedMechanismMode::kLegacyIntake &&
    indexed_mode != IndexedMechanismMode::kSortIntake) {
    start_wait_frame = -1;
    start_sort_frame = -1;
    return;
  }

  if(target == vex::color::red){
    target_hue = red_hue;
  }
  if(target == vex::color::blue){
    target_hue = blue_hue;
  }

  if (indexed_mode == IndexedMechanismMode::kLegacyIntake) {
    if (hue_within_range(hardware,target_hue,5)) {
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
