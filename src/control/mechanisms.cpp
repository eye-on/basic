#include "control/mechanisms.h"
#include "control/motor_control.h"
#include "control/autonomous/routine.h"

#include <array>

namespace basic::hardware::robots {

namespace {
//trans1 trans2 trans3 trans4 under middle upper
constexpr std::size_t kIndexedMotorCount = 6;
constexpr std::array<int, kIndexedMotorCount> kOffSpeeds{{0, 0, 0, 0, 0 ,0}};
constexpr std::array<int, kIndexedMotorCount> kPreLoadSpeeds{{-100, -100, 0, 0, 0, 0}};
constexpr std::array<int, kIndexedMotorCount> kLegacyIntakeSpeeds{{0, -80, 40, -100, 50, -50}};
constexpr std::array<int, kIndexedMotorCount> kUnderThrowSpeeds{{-100, -100, 0, 100, 0, 0}};
constexpr std::array<int, kIndexedMotorCount> kMiddleThrowSpeeds{{-100, -100, 100, -100, -100, 0}};
constexpr std::array<int, kIndexedMotorCount> kUpperThrowSpeeds{{-100, -100, 100, -100, 100, 100}};
constexpr std::array<int, kIndexedMotorCount> kSortIntakeSpeeds{{0, -80, 90, -100, 100 ,-100}};

void set_motor_power(vex::motor& motor, double speed, vex::brakeType type = vex::coast) {
  if (speed) {
    velocitycontrol(motor, speed, vex::pct);
  } else {
    stopcontrol(motor, type);
  }
}

void toggle_indexed_mode(MechanismState& mechanism, IndexedMechanismMode requested_mode) {
  if (mechanism.indexed_mode == requested_mode) {
    mechanism.indexed_mode = IndexedMechanismMode::kOff;
  } else {
    mechanism.indexed_mode = requested_mode;
  }
}

const std::array<int, kIndexedMotorCount>& indexed_motor_speeds(const MechanismState& mechanism) {
  switch (mechanism.indexed_mode) {
    case IndexedMechanismMode::kPreLoad:
      return kPreLoadSpeeds;break;
    case IndexedMechanismMode::kLegacyIntake:
      return kLegacyIntakeSpeeds;break;
    case IndexedMechanismMode::kUnderTrow:
      return kUnderThrowSpeeds;break;
    case IndexedMechanismMode::kMiddleThrow:
      return kMiddleThrowSpeeds;break;
    case IndexedMechanismMode::kUpperThrow:
      return kUpperThrowSpeeds;break;
    case IndexedMechanismMode::kSortIntake:
      return kSortIntakeSpeeds;break;
    case IndexedMechanismMode::kOff:
    default:
      return kOffSpeeds;
  }
}

void apply_indexed_mode(RobotHardware& hardware, const MechanismState& mechanism) {
  static const std::array<vex::motor*, kIndexedMotorCount> motors{{
      &hardware.trans_motor1,
      &hardware.trans_motor2,
      &hardware.trans_motor3,
      &hardware.under_motor1,
      &hardware.middle_motor1,
      &hardware.upper_motor1,
  }};

  const std::array<int, kIndexedMotorCount>& speeds = indexed_motor_speeds(mechanism);
  for (std::size_t index = 0; index < kIndexedMotorCount; ++index) {
    set_motor_power(*motors[index], speeds[index]);
  }
}

void update_under_overhang(RobotHardware& hardware, const ControllerInputState& input) {
  if (input.x) {
    velocitycontrol(hardware.under_overhang_motor, -20, vex::pct);
  }
  if (input.b) {
    velocitycontrol(hardware.under_overhang_motor, 20, vex::pct);
  }
  if (!input.x && !input.b) {
    stopcontrol(hardware.under_overhang_motor, vex::hold);
  }
  if(input.down){
    
  }
}

void update_middle_overhang(RobotHardware& hardware, const ControllerInputState& input) {
  if (input.left) {
    velocitycontrol(hardware.middle_overhang_motor, 20, vex::pct);
  }
  if (input.right) {
    velocitycontrol(hardware.middle_overhang_motor, -20, vex::pct);
  }
  if (!input.left && !input.right) {
    stopcontrol(hardware.middle_overhang_motor, vex::hold);
  }
}

void update_upper_overhang(RobotHardware& hardware, const ControllerInputState& input) {
  if (input.x) {
    velocitycontrol(hardware.upper_overhang_motor, -50, vex::pct);
  }
  if (input.b) {
    velocitycontrol(hardware.upper_overhang_motor, 50, vex::pct);
  }
  if (!input.x && !input.b) {
    stopcontrol(hardware.upper_overhang_motor, vex::hold);
  }
}

void update_overhang(RobotHardware& hardware, RobotState& state, const ControllerInputState& input){
  if(input.press_down){
    autonomous::update_under_overhang_mode(hardware,state,false);
  }
  if(input.press_left){
    autonomous::partially_collapse_middle_overhang(hardware,state,false);
  }
  /*if(input.press_up){
    autonomous::update_upper_overhang_mode(hardware,state,false);
  }*/
}

}  // namespace

void mechanism_update(RobotHardware& hardware, RobotState& state) {
  const ControllerInputState& input = state.controller;
  MechanismState& mechanism = state.mechanism;
  /*update_upper_overhang(hardware, input);
  update_middle_overhang(hardware, input);
  update_under_overhang(hardware, input);*/

  update_upper_overhang(hardware, input);
  update_overhang(hardware,state,input);
  
  if (input.press_a) {
    toggle_indexed_mode(mechanism, IndexedMechanismMode::kLegacyIntake);
  }
  if(input.press_l2){
    toggle_indexed_mode(mechanism, IndexedMechanismMode::kUnderTrow);
  }
  if (input.press_l1) {
    toggle_indexed_mode(mechanism, IndexedMechanismMode::kMiddleThrow);
  }
  if (input.press_r1) {
    toggle_indexed_mode(mechanism, IndexedMechanismMode::kUpperThrow);
  }
  if (input.press_r2){
    toggle_indexed_mode(mechanism, IndexedMechanismMode::kPreLoad);
  }

  apply_indexed_mode(hardware, mechanism);
}

void disable_indexed_mode(RobotHardware& hardware, RobotState& state){
  MechanismState& mechanism = state.mechanism;
  state.mechanism.indexed_mode = IndexedMechanismMode::kOff;
  apply_indexed_mode(hardware, mechanism);
}

void enable_preload_mode(RobotHardware& hardware, RobotState& state){
  MechanismState& mechanism = state.mechanism;
  state.mechanism.indexed_mode = IndexedMechanismMode::kPreLoad;
  apply_indexed_mode(hardware, mechanism);
}

void enable_intake_mode(RobotHardware& hardware, RobotState& state){
  MechanismState& mechanism = state.mechanism;
  state.mechanism.indexed_mode = IndexedMechanismMode::kLegacyIntake;
  apply_indexed_mode(hardware, mechanism);
}

void enable_underthrow_mode(RobotHardware& hardware, RobotState& state){
  MechanismState& mechanism = state.mechanism;
  state.mechanism.indexed_mode = IndexedMechanismMode::kUnderTrow;
  apply_indexed_mode(hardware, mechanism);
}

void enable_middlethrow_mode(RobotHardware& hardware, RobotState& state){
  MechanismState& mechanism = state.mechanism;
  state.mechanism.indexed_mode = IndexedMechanismMode::kMiddleThrow;
  apply_indexed_mode(hardware, mechanism);
}

void enable_upperthrow_mode(RobotHardware& hardware, RobotState& state){
  MechanismState& mechanism = state.mechanism;
  state.mechanism.indexed_mode = IndexedMechanismMode::kUpperThrow;
  apply_indexed_mode(hardware, mechanism);
}

void apply_indexed_mode(RobotHardware& hardware, RobotState& state){
  apply_indexed_mode(hardware, state.mechanism);
}

}  // namespace basic::hardware::robots
