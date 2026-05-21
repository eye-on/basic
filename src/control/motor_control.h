#ifndef BASIC_SRC_CONTROL_MOTOR_CONTROL_H_
#define BASIC_SRC_CONTROL_MOTOR_CONTROL_H_

#include "vex.h"

namespace basic::hardware::robots {

inline constexpr double kMotorTorqueConstantNmPerAmp = 0.84;
inline constexpr double kMotorRatedVoltageVolts = 12.0;

void velocitycontrol(vex::motor& motor, double velocity);
void velocitycontrol(vex::motor& motor, double velocity, vex::percentUnits units);
void velocitycontrol(vex::motor& motor, double velocity, vex::velocityUnits units);

void torquecontrol(vex::motor& motor, double torque_nm);
void stopcontrol(vex::motor& motor, vex::brakeType mode = vex::coast);

double get_position(vex::motor& motor, vex::rotationUnits units = vex::deg);
double get_revolutions(vex::motor& motor);
double get_velocity(vex::motor& motor, vex::velocityUnits units = vex::rpm);
double get_velocity(vex::motor& motor, vex::percentUnits units);
double get_current(vex::motor& motor, vex::currentUnits units = vex::amp);
double get_current(vex::motor& motor, vex::percentUnits units);
double get_voltage(vex::motor& motor, vex::voltageUnits units = vex::volt);
double get_power(vex::motor& motor, vex::powerUnits units = vex::watt);
double get_torque(vex::motor& motor, vex::torqueUnits units = vex::Nm);
double get_efficiency(vex::motor& motor, vex::percentUnits units = vex::pct);
double get_temperature(vex::motor& motor, vex::temperatureUnits units = vex::celsius);
double get_temperature(vex::motor& motor, vex::percentUnits units);
vex::directionType get_direction(vex::motor& motor);
bool get_spinning(vex::motor& motor);
bool get_done(vex::motor& motor);
bool get_installed(vex::motor& motor);

}  // namespace basic::hardware::robots

#endif
