#include "hardware/sensors.h"
#include "hardware/robots/robot_state.h"

#include "v5_apiuser.h"

namespace basic::hardware::robots {

namespace {
  const int red_hue = 7;
  const int blue_hue = 217;
  const int nothing_hue = 50;

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
                   const int wait_frames,       
                   const int continuous_frame) 
{
  IndexedMechanismMode& indexed_mode = state.mechanism.indexed_mode;

  static int current_frame = 0;
  ++current_frame;

  static int start_sort_frame = -1;
  static int start_wait_frame = -1;
  static bool prev_target_detected = false;
  static int sort_required_frames = 0;

    if (indexed_mode != IndexedMechanismMode::kLegacyIntake &&
        indexed_mode != IndexedMechanismMode::kSortIntake) {
        start_sort_frame = -1;
        start_wait_frame = -1;
        prev_target_detected = false;
        return;
    }
    int target_hue = -1;
    if (target == vex::color::red) {
        target_hue = red_hue;
    } else if (target == vex::color::blue) {
        target_hue = blue_hue;
    } else {
        return;
    }

    const int tolerance = 5;
    bool target_detected = hue_within_range(hardware, target_hue, tolerance);

    if (indexed_mode == IndexedMechanismMode::kLegacyIntake) {
        if (target_detected) {
            if (start_wait_frame == -1) {
              start_wait_frame = current_frame;  
            }
            if (current_frame - start_wait_frame >= wait_frames) {
              indexed_mode = IndexedMechanismMode::kSortIntake;
              start_sort_frame = current_frame;   
              start_wait_frame = -1;
              sort_required_frames = continuous_frame;              
            }
        } else {
          start_wait_frame = -1;
        }
    }
    else if (indexed_mode == IndexedMechanismMode::kSortIntake) {
        if (target_detected && !prev_target_detected) {
            start_sort_frame = current_frame;
            sort_required_frames = continuous_frame; 
        }
        if (!target_detected && sort_required_frames > continuous_frame / 2) {
            sort_required_frames = continuous_frame / 2;
        }
        if (current_frame - start_sort_frame >= sort_required_frames) {
            indexed_mode = IndexedMechanismMode::kLegacyIntake;
            start_sort_frame = -1;
            start_wait_frame = -1;
            sort_required_frames = 0; 
            prev_target_detected = target_detected;
            return;
        }
  }
  prev_target_detected = target_detected;
}

void count_balls_number(RobotHardware& hardware, RobotState& state){
  IndexedMechanismMode mode = state.mechanism.indexed_mode;

  static bool prev_red_detected  = false;
  static bool prev_blue_detected = false;

  const int tolerance = 5;

  bool red_detected  = hue_within_range(hardware, red_hue, tolerance);
  bool blue_detected = hue_within_range(hardware, blue_hue, tolerance);

  bool is_intake_or_sort = (mode == IndexedMechanismMode::kLegacyIntake ||
                            mode == IndexedMechanismMode::kSortIntake);

  if (red_detected && !prev_red_detected) {
    if (is_intake_or_sort) {
      state.red_balls++;
    } else {
      state.red_balls--;
    }
  }

  if (blue_detected && !prev_blue_detected) {
    if (is_intake_or_sort) {
      state.blue_balls++;
    } else {
      state.blue_balls--;
    }
  }

  prev_red_detected  = red_detected;
  prev_blue_detected = blue_detected;
}

}// namespace basic::hardware::robots
