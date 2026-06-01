#include "control/second_robot/autonomous/routine.h"

#include "control/motor_control.h"
#include "control/second_robot/mechanisms/mechanisms.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace basic::control::second_robot::autonomous {

namespace {

using basic::control::stopcontrol;
using basic::control::velocitycontrol;
using basic::hardware::second_robot::RobotHardware;
using basic::hardware::second_robot::RobotState;
using basic::hardware::second_robot::ShooterMode;

constexpr double kMillimetersPerWheelRevolution = 194.47;
constexpr double kAutonomousLoopDelayMs = 5.0;
constexpr double kAutonomousSettleDelayMs = 150.0;
constexpr double kDriveToleranceMm = 8.0;
constexpr double kDriveMinSpeedPct = 8.0;
constexpr double kDriveHeadingGain = 0.45;
constexpr double kDriveHeadingCorrectionLimitPct = 8.0;
constexpr double kTurnToleranceDeg = 1.0;
constexpr double kTurnGain = 0.7;
constexpr double kTurnApproachMinSpeedPct = 4.0;
constexpr double kTurnMinSpeedPct = 8.0;
constexpr double kTurnMaxSpeedPct = 30.0;
constexpr int kDriveBaseTimeoutMs = 1000;
constexpr int kDriveTimeoutPerMm = 1;
constexpr int kTurnBaseTimeoutMs = 700;
constexpr int kTurnTimeoutPerDegreeMs = 8;

using DriveMotorArray = std::array<vex::motor*, 6>;
using SideMotorArray = std::array<vex::motor*, 3>;

DriveMotorArray drive_motors(RobotHardware& hardware) {
  return {{
      &hardware.left_front_motor,
      &hardware.left_middle_motor,
      &hardware.left_back_motor,
      &hardware.right_front_motor,
      &hardware.right_middle_motor,
      &hardware.right_back_motor,
  }};
}

SideMotorArray left_drive_motors(RobotHardware& hardware) {
  return {{
      &hardware.left_front_motor,
      &hardware.left_middle_motor,
      &hardware.left_back_motor,
  }};
}

SideMotorArray right_drive_motors(RobotHardware& hardware) {
  return {{
      &hardware.right_front_motor,
      &hardware.right_middle_motor,
      &hardware.right_back_motor,
  }};
}

bool should_run_autonomous(vex::competition& competition) {
  return competition.isEnabled() && competition.isAutonomous();
}

double clamp_value(double value, double min_value, double max_value) {
  return std::min(std::max(value, min_value), max_value);
}

double normalize_angle_deg(double angle_deg) {
  while (angle_deg >= 180.0) {
    angle_deg -= 360.0;
  }
  while (angle_deg < -180.0) {
    angle_deg += 360.0;
  }
  return angle_deg;
}

double average_revolutions(const SideMotorArray& motors) {
  double total = 0.0;
  for (vex::motor* motor : motors) {
    total += motor->position(vex::rev);
  }
  return total / static_cast<double>(motors.size());
}

void set_drive_speed_pct(RobotHardware& hardware, double left_pct, double right_pct) {
  for (vex::motor* motor : left_drive_motors(hardware)) {
    velocitycontrol(*motor, left_pct, vex::pct);
  }
  for (vex::motor* motor : right_drive_motors(hardware)) {
    velocitycontrol(*motor, right_pct, vex::pct);
  }
}

void stop_drive(RobotHardware& hardware, vex::brakeType brake_type) {
  for (vex::motor* motor : drive_motors(hardware)) {
    stopcontrol(*motor, brake_type);
  }
}

double current_heading_deg(RobotHardware& hardware) {
  return normalize_angle_deg(hardware.inertial.rotation(vex::deg));
}

double average_distance_mm(RobotHardware& hardware) {
  const double left_mm = average_revolutions(left_drive_motors(hardware)) * kMillimetersPerWheelRevolution;
  const double right_mm = average_revolutions(right_drive_motors(hardware)) * kMillimetersPerWheelRevolution;
  return 0.5 * (left_mm + right_mm);
}

void settle_after_motion() {
  vex::this_thread::sleep_for(static_cast<int>(kAutonomousSettleDelayMs));
}

void set_toggle_state(vex::digital_out& output, bool value) {
  output.set(value);
}

}  // namespace

void drive_distance_mm(
    RobotHardware& hardware,
    RobotState& state,
    vex::competition& competition,
    double distance_mm,
    double max_speed_pct) {
  (void)state;
  if (!should_run_autonomous(competition) || distance_mm == 0.0) {
    return;
  }

  const double start_distance_mm = average_distance_mm(hardware);
  const double target_heading_deg = current_heading_deg(hardware);
  const int start_time_ms = hardware.brain.timer(vex::msec);
  const int timeout_ms =
      kDriveBaseTimeoutMs + static_cast<int>(std::ceil(std::fabs(distance_mm) * kDriveTimeoutPerMm));

  while (should_run_autonomous(competition)) {
    if (hardware.brain.timer(vex::msec) - start_time_ms >= timeout_ms) {
      break;
    }

    const double traveled_mm = average_distance_mm(hardware) - start_distance_mm;
    const double error_mm = distance_mm - traveled_mm;
    if (std::fabs(error_mm) <= kDriveToleranceMm) {
      break;
    }

    const double command_sign = error_mm >= 0.0 ? 1.0 : -1.0;
    double drive_pct = clamp_value(std::fabs(error_mm) * 0.08, kDriveMinSpeedPct, max_speed_pct);
    drive_pct *= command_sign;

    const double heading_error_deg = normalize_angle_deg(target_heading_deg - current_heading_deg(hardware));
    const double heading_correction_pct = clamp_value(
        heading_error_deg * kDriveHeadingGain,
        -kDriveHeadingCorrectionLimitPct,
        kDriveHeadingCorrectionLimitPct);

    set_drive_speed_pct(
        hardware,
        drive_pct + heading_correction_pct,
        drive_pct - heading_correction_pct);
    vex::this_thread::sleep_for(static_cast<int>(kAutonomousLoopDelayMs));
  }

  stop_drive(hardware, vex::hold);
  settle_after_motion();
}

void turn_to_heading_deg(
    RobotHardware& hardware,
    RobotState& state,
    vex::competition& competition,
    double target_heading_deg,
    double max_turn_speed_pct) {
  (void)state;
  if (!should_run_autonomous(competition)) {
    return;
  }

  const double normalized_target_deg = normalize_angle_deg(target_heading_deg);
  const int start_time_ms = hardware.brain.timer(vex::msec);
  const int timeout_ms =
      kTurnBaseTimeoutMs +
      static_cast<int>(std::ceil(std::fabs(normalize_angle_deg(normalized_target_deg - current_heading_deg(hardware))) *
                                 kTurnTimeoutPerDegreeMs));

  while (should_run_autonomous(competition)) {
    if (hardware.brain.timer(vex::msec) - start_time_ms >= timeout_ms) {
      break;
    }

    const double error_deg = normalize_angle_deg(normalized_target_deg - current_heading_deg(hardware));
    if (std::fabs(error_deg) <= kTurnToleranceDeg) {
      break;
    }

    const double min_speed_pct =
        std::fabs(error_deg) <= 12.0 ? kTurnApproachMinSpeedPct : kTurnMinSpeedPct;
    const double turn_pct = error_deg >= 0.0
                                ? clamp_value(std::fabs(error_deg) * kTurnGain, min_speed_pct, max_turn_speed_pct)
                                : -clamp_value(std::fabs(error_deg) * kTurnGain, min_speed_pct, max_turn_speed_pct);

    set_drive_speed_pct(hardware, turn_pct, -turn_pct);
    vex::this_thread::sleep_for(static_cast<int>(kAutonomousLoopDelayMs));
  }

  stop_drive(hardware, vex::coast);
  settle_after_motion();
}

void run_routine(RobotHardware& hardware, RobotState& state, vex::competition& competition) {
  state.chassis.stop_brake_type = vex::hold;
  state.autonomous = basic::hardware::shared::AutonomousState{};
  hardware.inertial.resetRotation();

  drive_distance_mm(hardware, state, competition, 970.0, 50.0);
  turn_to_heading_deg(hardware, state, competition, 45.0, 12.0);
  basic::control::second_robot::set_shooter_mode(hardware, state, ShooterMode::kMiddleShot, 100.0);
  drive_distance_mm(hardware, state, competition, 70.0, 100.0);
  vex::this_thread::sleep_for(500);
  basic::control::second_robot::set_shooter_mode(hardware, state, ShooterMode::kOff, 0.0);

  drive_distance_mm(hardware, state, competition, -1230.0, 40.0);
  turn_to_heading_deg(hardware, state, competition, 180.0, 10.0);
  state.mechanism.descore_open = true;
  set_toggle_state(hardware.descore, state.mechanism.descore_open);
  vex::this_thread::sleep_for(300);
  basic::control::second_robot::set_shooter_mode(hardware, state, ShooterMode::kRoller, 80.0);
  drive_distance_mm(hardware, state, competition, 410.0, 30.0);
  vex::this_thread::sleep_for(500);
  basic::control::second_robot::set_shooter_mode(hardware, state, ShooterMode::kOff, 0.0);

  drive_distance_mm(hardware, state, competition, -550.0, 50.0);
  state.mechanism.descore_open = false;
  set_toggle_state(hardware.descore, state.mechanism.descore_open);
  vex::this_thread::sleep_for(300);
  turn_to_heading_deg(hardware, state, competition, -1.0, 10.0);
  drive_distance_mm(hardware, state, competition, 165.0, 70.0);
  basic::control::second_robot::set_shooter_mode(hardware, state, ShooterMode::kLongShot, 100.0);
  vex::this_thread::sleep_for(1000);
  basic::control::second_robot::set_shooter_mode(hardware, state, ShooterMode::kOff, 0.0);
  drive_distance_mm(hardware, state, competition, 50.0, 100.0);
  drive_distance_mm(hardware, state, competition, -50.0, 100.0);

  drive_distance_mm(hardware, state, competition, -600.0, 40.0);
  turn_to_heading_deg(hardware, state, competition, 105.0, 10.0);
  drive_distance_mm(hardware, state, competition, 1500.0, 70.0);

  stop_drive(hardware, vex::hold);
}

}  // namespace basic::control::second_robot::autonomous
