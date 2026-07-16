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

  const double total_deg = std::max({std::abs(fl_deg), std::abs(fr_deg),
                                     std::abs(bl_deg), std::abs(br_deg)});
  const double v_min = 10.0;
  const double ramp_ratio = 0.3;

  double pos = 0.0;

  while (pos < total_deg) {
    double remaining = total_deg - pos;

    double v = v_peak_pct;
    if (pos < total_deg * ramp_ratio) {
      v = v_min + (v_peak_pct - v_min) * (pos / (total_deg * ramp_ratio));
    } else if (remaining < total_deg * ramp_ratio) {
      v = v_min + (v_peak_pct - v_min) * (remaining / (total_deg * ramp_ratio));
    }
    v = std::max(v_min, std::min(v, v_peak_pct));

    auto sign = [](double deg) -> double {
      if (deg == 0.0) return 0.0;
      return deg > 0.0 ? 1.0 : -1.0;
    };

    basic::chassis::x_drive_set_output(hardware.x_chassis,
                                       sign(fl_deg) * v,
                                       sign(fr_deg) * v,
                                       sign(bl_deg) * v,
                                       sign(br_deg) * v,
                                       vex::hold);

    vex::this_thread::sleep_for(kDtMs);

    if (fl_deg != 0.0) pos = std::abs(fl_motor.position(vex::deg));
    else if (fr_deg != 0.0) pos = std::abs(fr_motor.position(vex::deg));
    else if (bl_deg != 0.0) pos = std::abs(bl_motor.position(vex::deg));
    else pos = std::abs(br_motor.position(vex::deg));
  }

  basic::chassis::x_drive_stop(hardware.x_chassis, vex::hold);
}

void straight_drive(RobotHardware& hardware, double distance_mm, double v_peak_pct) {
  double deg = mm_to_wheel_deg(distance_mm);
  trapezoidal_drive(hardware, deg, 0.0, 0.0, deg, v_peak_pct);
}

void lateral_drive(RobotHardware& hardware, double distance_mm, double v_peak_pct) {
  double deg = mm_to_wheel_deg(distance_mm);
  trapezoidal_drive(hardware, 0.0, deg, deg, 0.0, v_peak_pct);
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
  double deg = mm_to_wheel_deg(distance_mm);
  trapezoidal_drive(hardware, deg, 0.0, 0.0, deg, speed_pct);
}

void move_backward(RobotHardware& hardware, double distance_mm, double speed_pct) {
  double deg = mm_to_wheel_deg(distance_mm);
  trapezoidal_drive(hardware, -deg, 0.0, 0.0, -deg, speed_pct);
}

void move_left(RobotHardware& hardware, double distance_mm, double speed_pct) {
  double deg = mm_to_wheel_deg(distance_mm);
  trapezoidal_drive(hardware, 0.0, -deg, -deg, 0.0, speed_pct);
}

void move_right(RobotHardware& hardware, double distance_mm, double speed_pct) {
  double deg = mm_to_wheel_deg(distance_mm);
  trapezoidal_drive(hardware, 0.0, deg, deg, 0.0, speed_pct);
}

void run_routine(RobotHardware& hardware, RobotState& state, vex::competition& competition) {
  if (!competition.isEnabled() || !competition.isAutonomous()) return;

  basic::chassis::x_drive_stop(hardware.x_chassis, vex::hold);

  // 1. 直行 830mm
  straight_drive(hardware, 830.0, kVPeakPct);
  wait_while_enabled(competition, kPauseMs);

  // 2. 停 1s
  wait_while_enabled(competition, kLongPauseMs);

  // 3. 后退 580mm
  straight_drive(hardware, -580.0, kVPeakPct);
  wait_while_enabled(competition, kPauseMs);

  // 4. 左平移 580mm
  lateral_drive(hardware, -580.0, kVPeakPct);
  wait_while_enabled(competition, kPauseMs);

  // 5. 停 1s
  wait_while_enabled(competition, kLongPauseMs);

  // 6. 右平移 200mm
  lateral_drive(hardware, 200.0, kVPeakPct);
  wait_while_enabled(competition, kPauseMs);

  // 7. 直行 600mm
  straight_drive(hardware, 600.0, kVPeakPct);
}

}  // namespace basic::hardware::looklook::autonomous
