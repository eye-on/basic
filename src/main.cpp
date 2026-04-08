#include "main.h"
#include "button.h"
#include "chassis.h"
#include "parameters.h"
#include "robot_confing.h"
#include "sensor.h"
#include "user_control.h"
#include "vex.h"

namespace {
pros::Task* controller_task = nullptr;
pros::Task* chassis_task = nullptr;
pros::Task* sensor_task = nullptr;
pros::Task* driver_task = nullptr;

void start_background_tasks() {
  if (controller_task == nullptr) {
    controller_task = new pros::Task([] { button_updating_thread(); }, "buttons");
  }
  if (chassis_task == nullptr) {
    chassis_task = new pros::Task([] { chassis_updating_thread(); }, "chassis");
  }
  if (sensor_task == nullptr) {
    sensor_task = new pros::Task([] { runsensor(); }, "sensor");
  }
}
}  // namespace

void initialize() {
  pros::lcd::initialize();

  vex::timer time_begin;
  time_begin.system();

  Inertial.calibrate();
  waitUntil(!Inertial.isCalibrating());
  Inertial.resetHeading();
  Inertial.resetRotation();

  Controller.Screen.setCursor(2, 1);
  Controller.Screen.print("calibrated!");

  start_background_tasks();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  if (driver_task == nullptr) {
    driver_task = new pros::Task([] { user_control_thread(); }, "driver");
  }

  while (true) {
    pros::delay(20);
  }
}
