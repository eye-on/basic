#include "mechanism/single_pneumatic.h"

namespace basic::mechanism {

namespace {

void apply_output(SinglePneumatic& mechanism) {
  mechanism.output().set(mechanism.state().open);
}

}  // namespace

SinglePneumatic::SinglePneumatic(const SinglePneumaticConfig& config)
    : output_(config.output.port) {}

vex::digital_out& SinglePneumatic::output() { return output_; }

const vex::digital_out& SinglePneumatic::output() const { return output_; }

SinglePneumaticState& SinglePneumatic::state() { return state_; }

const SinglePneumaticState& SinglePneumatic::state() const { return state_; }

SinglePneumatic single_pneumatic_init(const SinglePneumaticConfig& config) {
  return SinglePneumatic(config);
}

SinglePneumaticCommand single_pneumatic_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input) {
  SinglePneumaticCommand command;
  command.toggle = input.press_a;
  return command;
}

void single_pneumatic_update(
    SinglePneumatic& mechanism,
    const SinglePneumaticCommand& command) {
  if (command.toggle) {
    mechanism.state().open = !mechanism.state().open;
  }
  if (command.has_set_open) {
    mechanism.state().open = command.set_open;
  }
  apply_output(mechanism);
}

void single_pneumatic_set_open(SinglePneumatic& mechanism, bool open) {
  mechanism.state().open = open;
  apply_output(mechanism);
}

void single_pneumatic_stop(SinglePneumatic& mechanism) {
  mechanism.state() = SinglePneumaticState{};
  apply_output(mechanism);
}

SinglePneumaticState& single_pneumatic_state(SinglePneumatic& mechanism) {
  return mechanism.state();
}

const SinglePneumaticState& single_pneumatic_state(const SinglePneumatic& mechanism) {
  return mechanism.state();
}

}  // namespace basic::mechanism
