#ifndef BASIC_INCLUDE_ROLLER_SHOOTER_H_
#define BASIC_INCLUDE_ROLLER_SHOOTER_H_

#include "device_config.h"
#include "hardware/shared/state_types.h"

namespace basic::mechanism {

enum class RollerShooterMode {
  kOff,
  kRoller,
  kMiddleShot,
  kLongShot,
};

struct RollerShooterConfig {
  basic::device::MotorConfig roller_lower_motor;
  basic::device::MotorConfig roller_middle_motor;
  basic::device::MotorConfig roller_upper_motor;
  basic::device::DigitalOutConfig descore;
  basic::device::DigitalOutConfig hook;
  basic::device::DigitalOutConfig store;
};

struct RollerShooterCommand {
  RollerShooterMode shooter_mode{RollerShooterMode::kOff};
  double shooter_speed_pct{0.0};
  bool toggle_descore{false};
  bool toggle_hook{false};
  bool toggle_store{false};
};

struct RollerShooterState {
  RollerShooterMode shooter_mode{RollerShooterMode::kOff};
  double shooter_speed_pct{0.0};
  bool descore_open{false};
  bool hook_open{false};
  bool store_open{false};
};

class RollerShooter {
 public:
  explicit RollerShooter(const RollerShooterConfig& config);

  vex::motor& roller_lower_motor();
  vex::motor& roller_middle_motor();
  vex::motor& roller_upper_motor();
  vex::digital_out& descore();
  vex::digital_out& hook();
  vex::digital_out& store();

  const vex::motor& roller_lower_motor() const;
  const vex::motor& roller_middle_motor() const;
  const vex::motor& roller_upper_motor() const;
  const vex::digital_out& descore() const;
  const vex::digital_out& hook() const;
  const vex::digital_out& store() const;

  RollerShooterState& state();
  const RollerShooterState& state() const;

 private:
  vex::motor roller_lower_motor_;
  vex::motor roller_middle_motor_;
  vex::motor roller_upper_motor_;
  vex::digital_out descore_;
  vex::digital_out hook_;
  vex::digital_out store_;
  RollerShooterState state_;
};

RollerShooter roller_shooter_init(const RollerShooterConfig& config);
RollerShooterCommand roller_shooter_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input);

void roller_shooter_update(RollerShooter& mechanism, const RollerShooterCommand& command);
void roller_shooter_set_mode(
    RollerShooter& mechanism,
    RollerShooterMode mode,
    double speed_pct);
void roller_shooter_set_descore(RollerShooter& mechanism, bool open);
void roller_shooter_set_hook(RollerShooter& mechanism, bool open);
void roller_shooter_set_store(RollerShooter& mechanism, bool open);
void roller_shooter_stop(RollerShooter& mechanism);

RollerShooterState& roller_shooter_state(RollerShooter& mechanism);
const RollerShooterState& roller_shooter_state(const RollerShooter& mechanism);

}  // namespace basic::mechanism

#endif
