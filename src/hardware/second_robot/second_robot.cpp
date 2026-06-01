#include "hardware/robot_selector.h"

#include "control/motor_control.h"
#include "control/second_robot/autonomous/routine.h"
#include "control/second_robot/chassis/chassis.h"
#include "control/second_robot/mechanisms/mechanisms.h"
#include "hardware/second_robot/robot_hardware.h"
#include "hardware/second_robot/robot_state.h"
#include "hardware/second_robot/sensors.h"
#include "input/controller.h"

#include <array>

namespace basic::hardware::second_robot {

namespace {

class SecondRobot;
SecondRobot& current_second_robot();

class SecondRobot final : public basic::app::Robot {
 public:
  void initialize() override {
    hardware_.calibrate_inertial_sensor();
    hardware_.show_calibrated();
  }

  void bind_background_tasks() override {
    vex::thread background(start_background_tasks);
  }

  void bind_competition(vex::competition& competition) override {
    competition_ = &competition;
    competition.autonomous(start_autonomous_entry);
    competition.drivercontrol(start_driver_control_entry);
  }

 private:
  static void start_background_tasks() {
    current_second_robot().run_background_tasks();
  }

  static void start_driver_control_entry() {
    current_second_robot().run_driver_control_loop();
  }

  static void start_autonomous_entry() {
    current_second_robot().run_autonomous_routine();
  }

  void run_background_tasks() {
    while (true) {
      sensor_update(hardware_, state_);
      vex::this_thread::sleep_for(kSensorLoopDelay);
    }
  }

  void run_driver_control_loop() {
    while (should_run_driver_control()) {
      basic::input::controller_update(hardware_.brain, hardware_.controller, state_.controller);
      basic::control::second_robot::chassis_update(hardware_, state_);
      basic::control::second_robot::mechanism_update(hardware_, state_);
      vex::this_thread::sleep_for(kRefreshTime);
    }

    stop_all_outputs(vex::coast);
  }

  void run_autonomous_routine() {
    if (competition_ == nullptr) {
      return;
    }

    basic::control::second_robot::autonomous::run_routine(hardware_, state_, *competition_);
    stop_all_outputs(vex::hold);
  }

  bool should_run_driver_control() const {
    return competition_ != nullptr && competition_->isEnabled() && competition_->isDriverControl();
  }

  void stop_all_outputs(vex::brakeType drive_brake_type) {
    state_.controller = basic::hardware::shared::ControllerInputState{};
    state_.chassis = ChassisState{};
    state_.chassis.stop_brake_type = drive_brake_type;
    state_.mechanism = MechanismState{};

    const std::array<vex::motor*, 6> drive_motors{{
        &hardware_.left_front_motor,
        &hardware_.left_middle_motor,
        &hardware_.left_back_motor,
        &hardware_.right_front_motor,
        &hardware_.right_middle_motor,
        &hardware_.right_back_motor,
    }};
    for (vex::motor* motor : drive_motors) {
      basic::control::stopcontrol(*motor, drive_brake_type);
    }

    basic::control::stopcontrol(hardware_.roller_lower_motor, vex::coast);
    basic::control::stopcontrol(hardware_.roller_middle_motor, vex::coast);
    basic::control::stopcontrol(hardware_.roller_upper_motor, vex::coast);

    hardware_.descore.set(false);
    hardware_.hook.set(false);
    hardware_.store.set(false);
  }

  RobotHardware hardware_;
  RobotState state_;
  vex::competition* competition_{nullptr};

  friend SecondRobot& current_second_robot();
};

SecondRobot& current_second_robot() {
  static SecondRobot robot;
  return robot;
}

}  // namespace

basic::app::Robot& get_robot() {
  return current_second_robot();
}

}  // namespace basic::hardware::second_robot
