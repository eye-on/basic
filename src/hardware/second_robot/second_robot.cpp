#include "hardware/robot_selector.h"

#include "hardware/second_robot/autonomous.h"
#include "hardware/second_robot/robot_hardware.h"
#include "hardware/second_robot/robot_state.h"
#include "hardware/second_robot/sensors.h"
#include "input/controller.h"
#include "chassis/second_chassis.h"
#include "mechanism/roller_shooter.h"

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
      basic::chassis::second_chassis_update(
          hardware_.second_chassis,
          basic::chassis::second_chassis_command_from_controller(
              state_.controller,
              basic::chassis::second_chassis_state(hardware_.second_chassis).stop_brake_type));
      basic::mechanism::roller_shooter_update(
          hardware_.shooter,
          basic::mechanism::roller_shooter_command_from_controller(state_.controller));
      vex::this_thread::sleep_for(kRefreshTime);
    }

    stop_all_outputs(vex::coast);
  }

  void run_autonomous_routine() {
    if (competition_ == nullptr) {
      return;
    }

    basic::hardware::second_robot::autonomous::run_routine(hardware_, state_, *competition_);
    stop_all_outputs(vex::hold);
  }

  bool should_run_driver_control() const {
    return competition_ != nullptr && competition_->isEnabled() && competition_->isDriverControl();
  }

  void stop_all_outputs(vex::brakeType drive_brake_type) {
    state_.controller = basic::hardware::shared::ControllerInputState{};
    basic::chassis::second_chassis_stop(hardware_.second_chassis, drive_brake_type);
    basic::mechanism::roller_shooter_stop(hardware_.shooter);
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
