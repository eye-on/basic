#include "mechanism/dual_pneumatic.h"

namespace basic::mechanism {

namespace {

void apply_output(DualPneumatic& mechanism) {
  const bool engaged =
      mechanism.state().mode == DualPneumaticMode::kEngaged;
  const bool level = engaged == mechanism.config().engaged_signal;
  mechanism.cylinder_a().set(level);
  mechanism.cylinder_b().set(level);
  mechanism.state().output_signal = level;
}

}  // namespace

DualPneumatic::DualPneumatic(const DualPneumaticConfig& config)
    : cylinder_a_(config.cylinder_a.port),
      cylinder_b_(config.cylinder_b.port),
      config_(config) {
  state_.mode = config.initial_mode;  // 上电即置于初始状态（默认展开/松开）
  apply_output(*this);                // 构造时立刻下发两个电磁阀电平
}

vex::digital_out& DualPneumatic::cylinder_a() { return cylinder_a_; }
vex::digital_out& DualPneumatic::cylinder_b() { return cylinder_b_; }
const vex::digital_out& DualPneumatic::cylinder_a() const { return cylinder_a_; }
const vex::digital_out& DualPneumatic::cylinder_b() const { return cylinder_b_; }

DualPneumaticConfig& DualPneumatic::config() { return config_; }
const DualPneumaticConfig& DualPneumatic::config() const { return config_; }

DualPneumaticState& DualPneumatic::state() { return state_; }
const DualPneumaticState& DualPneumatic::state() const { return state_; }

DualPneumatic dual_pneumatic_init(const DualPneumaticConfig& config) {
  return DualPneumatic(config);
}

DualPneumaticCommand dual_pneumatic_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input) {
  DualPneumaticCommand command;
  command.toggle = input.press_b;  // B 按下沿 → 翻转 框住/松开
  return command;
}

void dual_pneumatic_update(
    DualPneumatic& mechanism,
    const DualPneumaticCommand& command) {
  if (command.toggle) {
    mechanism.state().mode =
        (mechanism.state().mode == DualPneumaticMode::kEngaged)
            ? DualPneumaticMode::kReleased
            : DualPneumaticMode::kEngaged;
    ++mechanism.state().toggle_count;
  }
  apply_output(mechanism);
}

void dual_pneumatic_set_mode(
    DualPneumatic& mechanism,
    DualPneumaticMode mode) {
  mechanism.state().mode = mode;
  apply_output(mechanism);
}

void dual_pneumatic_stop(DualPneumatic& mechanism) {
  mechanism.state() = DualPneumaticState{};
  apply_output(mechanism);
}

DualPneumaticState& dual_pneumatic_state(DualPneumatic& mechanism) {
  return mechanism.state();
}

const DualPneumaticState& dual_pneumatic_state(
    const DualPneumatic& mechanism) {
  return mechanism.state();
}

}  // namespace basic::mechanism
