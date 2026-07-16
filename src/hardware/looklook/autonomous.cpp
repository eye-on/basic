#include "hardware/looklook/autonomous.h"

#include "chassis/x_chassis.h"

namespace basic::hardware::looklook::autonomous {

namespace {

inline constexpr double kVPeakPct = 15.0;
inline constexpr double kTurnPct = 25.0;
inline constexpr int kPauseMs = 300;
inline constexpr int kLongPauseMs = 1000;
inline constexpr double kTurnToleranceDeg = 3.0;
inline constexpr int kDtMs = 10;

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
    vex::this_thread::sleep_for(kDtMs);
  }
}

void trapezoidal_drive(RobotHardware& hardware,
                       double fl_deg, double fr_deg, double bl_deg, double br_deg,
                       double v_peak_pct) {
  auto& fl_motor = basic::chassis::x_chassis_fl(hardware.x_chassis);
  auto& fr_motor = basic::chassis::x_chassis_fr(hardware.x_chassis);
  auto& bl_motor = basic::chassis::x_chassis_bl(hardware.x_chassis);
  auto& br_motor = basic::chassis::x_chassis_br(hardware.x_chassis);

  fl_motor.resetPosition();
  fr_motor.resetPosition();
  bl_motor.resetPosition();
  br_motor.resetPosition();

  const double total_deg = std::abs(fl_deg);
  const double sign = (fl_deg > 0.0) ? 1.0 : -1.0;
  const double v_min = 10.0;
  const double accel = 60.0;  // pct/s ramp rate
  const double dt_s = static_cast<double>(kDtMs) * 0.001;

  double v = v_min;
  double pos = 0.0;

  while (pos < total_deg) {
    double remaining = total_deg - pos;

    if (pos < total_deg * 0.3) {
      v = v_min + (v_peak_pct - v_min) * (pos / (total_deg * 0.3));
    } else if (remaining < total_deg * 0.3) {
      v = v_min + (v_peak_pct - v_min) * (remaining / (total_deg * 0.3));
    } else {
      v = v_peak_pct;
    }

    v = std::max(v_min, std::min(v, v_peak_pct));

    double v_cmd = sign * v;
    basic::chassis::x_drive_set_output(hardware.x_chassis,
                                       v_cmd, v_cmd, v_cmd, v_cmd, vex::hold);

    vex::this_thread::sleep_for(kDtMs);
    pos = std::abs(fl_motor.position(vex::deg));
  }

  basic::chassis::x_drive_stop(hardware.x_chassis, vex::hold);
}

void turn_to(RobotHardware& hardware, double delta_deg) {
  double target = hardware.inertial.heading(vex::deg) + delta_deg;

  while (true) {
    double error = target - hardware.inertial.heading(vex::deg);
    while (error > 180.0) error -= 360.0;
    while (error < -180.0) error += 360.0;

    if (std::abs(error) < kTurnToleranceDeg) break;

    double t = (error > 0.0 ? 1.0 : -1.0) * kTurnPct;
    basic::chassis::x_drive_set_output(hardware.x_chassis, t, -t, t, -t, vex::hold);
    vex::this_thread::sleep_for(kDtMs);
  }

  basic::chassis::x_drive_stop(hardware.x_chassis, vex::hold);
}

void wait_while_enabled(vex::competition& comp, int ms) {
  int elapsed = 0;
  while (elapsed < ms && comp.isEnabled() && comp.isAutonomous()) {
    vex::this_thread::sleep_for(kDtMs);
    elapsed += kDtMs;
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
  if (!competition.isEnabled() || !competition.isAutonomous()) return;

  basic::chassis::x_drive_stop(hardware.x_chassis, vex::hold);

  // 1. 直行 775mm (= 1200 - 425)
  {
    double deg = mm_to_wheel_deg(1200.0 - 425.0);
    trapezoidal_drive(hardware, deg, deg, deg, deg, kVPeakPct);
  }
  wait_while_enabled(competition, kPauseMs);

  // 2. 倒退 387.5mm
  {
    double deg = mm_to_wheel_deg(387.5);
    trapezoidal_drive(hardware, -deg, -deg, -deg, -deg, kVPeakPct);
  }
  wait_while_enabled(competition, kPauseMs);

  // 3. 左转 90°
  turn_to(hardware, 90.0);
  wait_while_enabled(competition, kPauseMs);

  // 4. 直行 387.5mm
  {
    double deg = mm_to_wheel_deg(387.5);
    trapezoidal_drive(hardware, deg, deg, deg, deg, kVPeakPct);
  }
  wait_while_enabled(competition, kPauseMs);

  // 5. 停 1s
  wait_while_enabled(competition, kLongPauseMs);

  // 6. 后退 387.5mm
  {
    double deg = mm_to_wheel_deg(387.5);
    trapezoidal_drive(hardware, -deg, -deg, -deg, -deg, kVPeakPct);
  }
  wait_while_enabled(competition, kPauseMs);

  // 7. 右转 90°
  turn_to(hardware, -90.0);
  wait_while_enabled(competition, kPauseMs);

  // 8. 直行 600mm
  {
    double deg = mm_to_wheel_deg(600.0);
    trapezoidal_drive(hardware, deg, deg, deg, deg, kVPeakPct);
  }
}

}  // namespace basic::hardware::looklook::autonomous
