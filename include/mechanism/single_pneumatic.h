#ifndef BASIC_INCLUDE_MECHANISM_SINGLE_PNEUMATIC_H_
#define BASIC_INCLUDE_MECHANISM_SINGLE_PNEUMATIC_H_

#include "device_config.h"
#include "hardware/shared/state_types.h"

namespace basic::mechanism {

struct SinglePneumaticConfig {
  basic::device::DigitalOutConfig output;
};

struct SinglePneumaticCommand {
  bool toggle{false};
  bool set_open{false};
  bool has_set_open{false};
};

struct SinglePneumaticState {
  bool open{false};
};

class SinglePneumatic {
 public:
  explicit SinglePneumatic(const SinglePneumaticConfig& config);

  vex::digital_out& output();
  const vex::digital_out& output() const;

  SinglePneumaticState& state();
  const SinglePneumaticState& state() const;

 private:
  vex::digital_out output_;
  SinglePneumaticState state_;
};

SinglePneumatic single_pneumatic_init(const SinglePneumaticConfig& config);

SinglePneumaticCommand single_pneumatic_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input);

void single_pneumatic_update(
    SinglePneumatic& mechanism,
    const SinglePneumaticCommand& command);

void single_pneumatic_set_open(SinglePneumatic& mechanism, bool open);
void single_pneumatic_stop(SinglePneumatic& mechanism);

SinglePneumaticState& single_pneumatic_state(SinglePneumatic& mechanism);
const SinglePneumaticState& single_pneumatic_state(const SinglePneumatic& mechanism);

}  // namespace basic::mechanism

#endif
