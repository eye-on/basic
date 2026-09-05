#include "mechanism/pneumatic_gripper.h"

namespace basic::mechanism {

namespace {

void apply_output(PneumaticGripper& mechanism) {
  // 电平 = (抓握) XOR (inverted)：抓握是否对应高电平由接线决定
  const bool grasp_high =
      mechanism.state().mode == PneumaticGripperMode::kGrasp;
  mechanism.output().set(grasp_high != mechanism.config().inverted);
}

}  // namespace

PneumaticGripper::PneumaticGripper(const PneumaticGripperConfig& config)
    : config_(config), output_(config.output.port) {}

vex::digital_out& PneumaticGripper::output() { return output_; }
const vex::digital_out& PneumaticGripper::output() const { return output_; }

PneumaticGripperConfig& PneumaticGripper::config() { return config_; }
const PneumaticGripperConfig& PneumaticGripper::config() const {
  return config_;
}

PneumaticGripperState& PneumaticGripper::state() { return state_; }
const PneumaticGripperState& PneumaticGripper::state() const { return state_; }

PneumaticGripper pneumatic_gripper_init(const PneumaticGripperConfig& config) {
  return PneumaticGripper(config);
}

PneumaticGripperCommand pneumatic_gripper_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input) {
  PneumaticGripperCommand command;
  command.toggle = input.press_r1;  // R1 按下沿 → 翻转 抓握/松开
  return command;
}

void pneumatic_gripper_update(
    PneumaticGripper& mechanism,
    const PneumaticGripperCommand& command) {
  // 边沿触发：按一下切换状态（长按不重复触发）
  if (command.toggle) {
    mechanism.state().mode =
        (mechanism.state().mode == PneumaticGripperMode::kGrasp)
            ? PneumaticGripperMode::kRelease
            : PneumaticGripperMode::kGrasp;
  }
  apply_output(mechanism);
}

void pneumatic_gripper_set_state(
    PneumaticGripper& mechanism,
    PneumaticGripperMode state) {
  mechanism.state().mode = state;
  apply_output(mechanism);
}

void pneumatic_gripper_stop(PneumaticGripper& mechanism) {
  mechanism.state() = PneumaticGripperState{};
  apply_output(mechanism);  // 复位为松开并切断电磁阀
}

PneumaticGripperState& pneumatic_gripper_state(PneumaticGripper& mechanism) {
  return mechanism.state();
}

const PneumaticGripperState& pneumatic_gripper_state(
    const PneumaticGripper& mechanism) {
  return mechanism.state();
}

}  // namespace basic::mechanism
