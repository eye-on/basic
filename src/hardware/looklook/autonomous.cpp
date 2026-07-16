#include "hardware/looklook/autonomous.h"

#include "chassis/x_chassis.h"

namespace basic::hardware::looklook::autonomous {

namespace {

double mm_to_wheel_deg(double distance_mm) {
  return distance_mm * kWheelDegPerMm;
}

bool all_motors_at_target(vex::motor& fl, vex::motor& fr, vex::motor& bl, vex::motor& br) {
  return !fl.isSpinning() && !fr.isSpinning() && !bl.isSpinning() && !br.isSpinning();
}

void drive_to_positions(RobotHardware& hardware,
                        double fl_deg, double fr_deg, double bl_deg, double br_deg,
                        double speed_pct) {
  auto& fl_motor = basic::chassis::x_chassis_fl(hardware.x_chassis);
  auto& fr_motor = basic::chassis::x_chassis_fr(hardware.x_chassis);
  auto& bl_motor = basic::chassis::x_chassis_bl(hardware.x_chassis);
  auto& br_motor = basic::chassis::x_chassis_br(hardware.x_chassis);

  fl_motor.resetPosition();
  fr_motor.resetPosition();
  bl_motor.resetPosition();
  br_motor.resetPosition();

  fl_motor.spinToPosition(fl_deg, vex::deg, speed_pct, vex::velocityUnits::pct, false);
  fr_motor.spinToPosition(fr_deg, vex::deg, speed_pct, vex::velocityUnits::pct, false);
  bl_motor.spinToPosition(bl_deg, vex::deg, speed_pct, vex::velocityUnits::pct, false);
  br_motor.spinToPosition(br_deg, vex::deg, speed_pct, vex::velocityUnits::pct, false);

  while (!all_motors_at_target(fl_motor, fr_motor, bl_motor, br_motor)) {
    vex::this_thread::sleep_for(kRefreshTimeMs);
  }
}

}  // namespace

void move_forward(RobotHardware& hardware, double distance_mm, double speed_pct) {
  const double deg = mm_to_wheel_deg(distance_mm);
  drive_to_positions(hardware, deg, deg, deg, deg, speed_pct);
}

void move_backward(RobotHardware& hardware, double distance_mm, double speed_pct) {
  const double deg = mm_to_wheel_deg(-distance_mm);
  drive_to_positions(hardware, deg, deg, deg, deg, speed_pct);
}

void move_left(RobotHardware& hardware, double distance_mm, double speed_pct) {
  const double deg = mm_to_wheel_deg(distance_mm);
  drive_to_positions(hardware, -deg, deg, deg, -deg, speed_pct);
}

void move_right(RobotHardware& hardware, double distance_mm, double speed_pct) {
  const double deg = mm_to_wheel_deg(distance_mm);
  drive_to_positions(hardware, deg, -deg, -deg, deg, speed_pct);
}

void run_routine(RobotHardware& hardware, RobotState& state, vex::competition& competition) {

}

}  // namespace basic::hardware::looklook::autonomous
