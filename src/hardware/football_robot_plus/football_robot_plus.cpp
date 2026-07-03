#include "hardware/robot_selector.h"

#include "chassis/x_chassis.h"
#include "hardware/football_robot_plus/robot_hardware.h"
#include "hardware/football_robot_plus/robot_state.h"
#include "hardware/football_robot_plus/runtime.h"
#include "hardware/football_robot_plus/sensors.h"
#include "hardware/football_robot_plus/vision.h"
#include "input/controller.h"

#include <cmath>

namespace basic::hardware::football_robot_plus {

namespace autonomous {
void step(RobotHardware& hardware, RobotState& state, RuntimeState& runtime);
}

namespace {

constexpr int kCameraGimbalInputDeadzonePct = 5;

class FootballRobotPlus;
FootballRobotPlus& current_football_robot_plus();

class FootballRobotPlus final : public basic::app::Robot {
 public:
  void initialize() override {
    sensors::configure_vision(
        hardware_,
        state_,
        runtime_,
        sensors::default_vision_config_for_sensor());
    sensors::refresh_camera_gimbal_state(hardware_, state_);
    sensors::refresh_dual_motor_actuator_state(hardware_, state_);
    hardware_.calibrate_inertial_sensor();
    hardware_.inertial.resetRotation();
    hardware_.inertial.resetHeading();
    basic::chassis::x_chassis_reset_odometry(hardware_.football_chassis, 0.0, 0.0);
    state_.autonomous = basic::hardware::shared::AutonomousState{};
    state_.autonomous.initialized = true;
    state_.autonomous.target_heading_deg = 0.0;
    state_.autonomous.estimated_heading_deg = 0.0;
    state_.autonomous.estimated_x_mm = 0.0;
    state_.autonomous.estimated_y_mm = 0.0;
    hardware_.external_vision.initialize();
    sensors::set_vision_target_color(state_, basic::identify::VisionTargetColor::kRed);
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

  void configure_vision(const FootballVisionConfig& config) {
    sensors::configure_vision(hardware_, state_, runtime_, config);
  }

  void set_vision_target_color(basic::identify::VisionTargetColor color) {
    sensors::set_vision_target_color(state_, color);
  }

  basic::identify::VisionTargetColor vision_target_color() const {
    return state_.vision.target_color;
  }

  basic::identify::LargestBlobDetection vision_sensor_detection() const {
    return state_.vision.last_blob_detection;
  }

  basic::vision::EstimateResult submit_yolo_detection(const YoloDetection& detection) {
    return sensors::submit_yolo_detection(hardware_, state_, runtime_, detection);
  }

  void clear_yolo_detection() {
    sensors::clear_yolo_detection(hardware_, state_);
  }

  FootballVisionState vision_state() const { return state_.vision; }

 private:
  static void start_background_tasks() {
    current_football_robot_plus().run_background_tasks();
  }

  static void start_driver_control_entry() {
    current_football_robot_plus().run_driver_control_loop();
  }

  static void start_autonomous_entry() {
    current_football_robot_plus().run_autonomous_routine();
  }

  void run_background_tasks() {
    while (true) {
      basic::input::controller_update(hardware_.brain, hardware_.controller, state_.controller);
      sensors::update(hardware_, state_, runtime_);

      if (runtime_.auto_mode == AutoMode::kManual && should_accept_manual_control()) {
        run_camera_gimbal_manual_step();
      } else if (runtime_.auto_mode != AutoMode::kIntercept) {
        stop_camera_gimbal(vex::hold);
      }

      autonomous::step(hardware_, state_, runtime_);

      if (runtime_.auto_mode == AutoMode::kManual) {
        if (should_accept_manual_control()) {
          run_manual_control_step();
        } else {
          stop_all_outputs(vex::coast);
        }
      }

      vex::this_thread::sleep_for(kRefreshTime);
    }
  }

  void run_driver_control_loop() {
    while (should_run_driver_control()) {
      vex::this_thread::sleep_for(kRefreshTime);
    }
  }

  void run_autonomous_routine() {
    while (should_run_autonomous_callback()) {
      vex::this_thread::sleep_for(kRefreshTime);
    }
  }

  bool should_run_driver_control() const {
    return competition_ != nullptr && competition_->isEnabled() &&
           competition_->isDriverControl();
  }

  bool should_run_autonomous_callback() const {
    return competition_ != nullptr && competition_->isEnabled() &&
           competition_->isAutonomous();
  }

  bool should_accept_manual_control() const {
    return competition_ == nullptr || !competition_->isEnabled() ||
           competition_->isDriverControl();
  }

  void run_camera_gimbal_manual_step() {
    const int axis_input_pct = state_.controller.axis3;
    if (std::abs(axis_input_pct) < kCameraGimbalInputDeadzonePct) {
      basic::mechanism::camera_gimbal_stop(hardware_.camera_gimbal, vex::hold);
      sensors::refresh_camera_gimbal_state(hardware_, state_);
      return;
    }

    basic::mechanism::camera_gimbal_set_output(
        hardware_.camera_gimbal,
        static_cast<double>(axis_input_pct));
    sensors::refresh_camera_gimbal_state(hardware_, state_);
  }

  void run_dual_motor_actuator_manual_step() {
    basic::mechanism::DualMotorActuatorCommand command;
    command.run_primary_sequence = state_.controller.press_x;
    command.toggle_secondary_target_state = state_.controller.press_a;
    if (!command.run_primary_sequence && !command.toggle_secondary_target_state) {
      return;
    }

    basic::mechanism::dual_motor_actuator_update(hardware_.dual_motor_actuator, command);
    sensors::refresh_dual_motor_actuator_state(hardware_, state_);
  }

  void run_manual_control_step() {
    run_dual_motor_actuator_manual_step();
    const basic::chassis::XChassisCommand command =
        basic::chassis::x_chassis_command_from_controller(
            state_.controller,
            basic::chassis::x_chassis_state(hardware_.football_chassis).stop_brake_type);
    basic::chassis::x_chassis_update(hardware_.football_chassis, command);
  }

  void stop_camera_gimbal(vex::brakeType brake_type) {
    basic::mechanism::camera_gimbal_stop(hardware_.camera_gimbal, brake_type);
    sensors::refresh_camera_gimbal_state(hardware_, state_);
  }

  void stop_all_outputs(vex::brakeType drive_brake_type) {
    basic::chassis::x_chassis_stop(hardware_.football_chassis, drive_brake_type);
    basic::mechanism::camera_gimbal_stop(hardware_.camera_gimbal, vex::hold);
    basic::mechanism::dual_motor_actuator_stop(hardware_.dual_motor_actuator, vex::hold);
    sensors::refresh_camera_gimbal_state(hardware_, state_);
    sensors::refresh_dual_motor_actuator_state(hardware_, state_);
  }

  RobotHardware hardware_;
  RobotState state_;
  RuntimeState runtime_{};
  vex::competition* competition_{nullptr};

  friend FootballRobotPlus& current_football_robot_plus();
};

FootballRobotPlus& current_football_robot_plus() {
  static FootballRobotPlus robot;
  return robot;
}

}  // namespace

basic::app::Robot& get_robot() {
  return current_football_robot_plus();
}

void configure_vision(const FootballVisionConfig& config) {
  current_football_robot_plus().configure_vision(config);
}

void set_vision_target_color(basic::identify::VisionTargetColor color) {
  current_football_robot_plus().set_vision_target_color(color);
}

basic::identify::VisionTargetColor get_vision_target_color() {
  return current_football_robot_plus().vision_target_color();
}

basic::identify::LargestBlobDetection get_vision_sensor_detection() {
  return current_football_robot_plus().vision_sensor_detection();
}

basic::vision::EstimateResult submit_yolo_detection(const YoloDetection& detection) {
  return current_football_robot_plus().submit_yolo_detection(detection);
}

void clear_yolo_detection() {
  current_football_robot_plus().clear_yolo_detection();
}

FootballVisionState get_vision_state() {
  return current_football_robot_plus().vision_state();
}

}  // namespace basic::hardware::football_robot_plus
