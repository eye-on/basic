#include "control/motor_control.h"

#include <cmath>

namespace basic::hardware::robots {

namespace {

vex::directionType direction_from_value(double value) {
  return value >= 0.0 ? vex::directionType::fwd : vex::directionType::rev;
}

double magnitude(double value) {
  return std::fabs(value);
}

}  // namespace

void velocitycontrol(vex::motor& motor, double velocity) {
  velocitycontrol(motor, velocity, vex::pct);
}

void velocitycontrol(vex::motor& motor, double velocity, vex::percentUnits units) {
  if (velocity == 0.0) {
    stopcontrol(motor);
    return;
  }

  motor.setMaxTorque(100.0, vex::pct);
  motor.spin(direction_from_value(velocity), magnitude(velocity), units);
}

void velocitycontrol(vex::motor& motor, double velocity, vex::velocityUnits units) {
  if (velocity == 0.0) {
    stopcontrol(motor);
    return;
  }

  motor.setMaxTorque(100.0, vex::pct);
  motor.spin(direction_from_value(velocity), magnitude(velocity), units);
}

void torquecontrol(vex::motor& motor, double torque_nm) {
  if (torque_nm == 0.0) {
    stopcontrol(motor);
    return;
  }

  // VEX does not expose direct torque control, so approximate it with
  // current limiting derived from the torque constant and rated-voltage drive.
  const double current_limit_amp = magnitude(torque_nm) / kMotorTorqueConstantNmPerAmp;
  motor.setMaxTorque(current_limit_amp, vex::amp);
  motor.spin(direction_from_value(torque_nm), kMotorRatedVoltageVolts, vex::volt);
}

void stopcontrol(vex::motor& motor, vex::brakeType mode) {
  motor.stop(mode);
}

double get_position(vex::motor& motor, vex::rotationUnits units) {
  return motor.position(units);
}

double get_revolutions(vex::motor& motor) {
  return motor.position(vex::rev);
}

double get_velocity(vex::motor& motor, vex::velocityUnits units) {
  return motor.velocity(units);
}

double get_velocity(vex::motor& motor, vex::percentUnits units) {
  return motor.velocity(units);
}

double get_current(vex::motor& motor, vex::currentUnits units) {
  return motor.current(units);
}

double get_current(vex::motor& motor, vex::percentUnits units) {
  return motor.current(units);
}

double get_voltage(vex::motor& motor, vex::voltageUnits units) {
  return motor.voltage(units);
}

double get_power(vex::motor& motor, vex::powerUnits units) {
  return motor.power(units);
}

double get_torque(vex::motor& motor, vex::torqueUnits units) {
  return motor.torque(units);
}

double get_efficiency(vex::motor& motor, vex::percentUnits units) {
  return motor.efficiency(units);
}

double get_temperature(vex::motor& motor, vex::temperatureUnits units) {
  return motor.temperature(units);
}

double get_temperature(vex::motor& motor, vex::percentUnits units) {
  return motor.temperature(units);
}

vex::directionType get_direction(vex::motor& motor) {
  return motor.direction();
}

bool get_spinning(vex::motor& motor) {
  return motor.isSpinning();
}

bool get_done(vex::motor& motor) {
  return motor.isDone();
}

bool get_installed(vex::motor& motor) {
  return motor.installed();
}

}  // namespace basic::hardware::robots
