#include "control/autonomous/routine.h"

#include "control/mechanisms.h"
#include "control/motor_control.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace basic::hardware::robots::autonomous {

namespace {

constexpr double kMillimetersPerWheelRevolution = 212.8;
constexpr double kPi = 3.14159265358979323846;
constexpr int kAutonomousLoopDelayMs = 10;
constexpr int kAutonomousSettleDelayMs = 150;

constexpr double kDriveMinSpeedPct = 12.0;
 double kDriveMaxSpeedPct = 22.5;
constexpr double kDriveAccelerationWindowMm = 180.0;
constexpr double kDriveDecelerationWindowMm = 260.0;
constexpr double kDriveHeadingProportionalGain = 0.6;
constexpr double kDriveHeadingCorrectionMaxPct = 4.0;
constexpr double kDriveHeadingCorrectionSpeedRatio = 0.2;
constexpr double kDriveHeadingDeadbandDegrees = 1.0;

constexpr double kLaserDistanceToleranceMm = 10.0;
constexpr double kLaserDistanceMinSpeedPct = 6.0;
constexpr double kLaserDistanceMaxSpeedPct = 20.0; //18
constexpr double kLaserDistanceAccelerationWindowMm = 120.0;
constexpr double kLaserDistanceDecelerationWindowMm = 180.0;
constexpr int kLaserDistanceBaseTimeoutMs = 1200;
constexpr int kLaserDistanceTimeoutPerMm = 4;

constexpr double kTurnToleranceDegrees = 1.5;
constexpr double kTurnProportionalGain = 0.6;
constexpr double kTurnMinSpeedPct = 10.0;
constexpr double kTurnApproachMinSpeedPct = 4.0;
constexpr double kTurnMaxSpeedPct = 30.0;
constexpr double kTurnApproachWindowDegrees = 12.0;

constexpr double kGoToPosePositionToleranceMm = 30.0;
constexpr double kGoToPoseHeadingToleranceDegrees = 3.0;
constexpr double kGoToPoseMinSpeedPct = 8.0;
constexpr double kGoToPoseMaxSpeedPct = 30.0; //24
constexpr double kGoToPoseAccelerationWindowMm = 180.0;
constexpr double kGoToPoseDecelerationWindowMm = 320.0;
constexpr double kGoToPoseLookaheadMinMm = 80.0;
constexpr double kGoToPoseLookaheadMaxMm = 220.0;
constexpr double kGoToPoseHeadingBlendWindowMm = 180.0;
constexpr double kGoToPoseHeadingSlowWindowDegrees = 70.0;
constexpr double kGoToPoseTurnProportionalGain = 0.9;
constexpr double kGoToPoseTurnMaxPct = 24.0;
constexpr int kGoToPoseBaseTimeoutMs = 1500;
constexpr int kGoToPoseTimeoutPerMm = 10;
constexpr int kGoToPoseTimeoutPerDegreeMs = 40;

using DriveMotorArray = std::array<vex::motor*, 8>;
using SideMotorArray = std::array<vex::motor*, 4>;

struct DriveSideRevolutions {
  double left_rev{0.0};
  double right_rev{0.0};
};

DriveMotorArray drive_motors(RobotHardware& hardware) {
  return {{
      &hardware.motor_fl1,
      &hardware.motor_fl2,
      &hardware.motor_fr1,
      &hardware.motor_fr2,
      &hardware.motor_bl1,
      &hardware.motor_bl2,
      &hardware.motor_br1,
      &hardware.motor_br2,
  }};
}

SideMotorArray left_drive_motors(RobotHardware& hardware) {
  return {{
      &hardware.motor_fl1,
      &hardware.motor_fl2,
      &hardware.motor_bl1,
      &hardware.motor_bl2,
  }};
}

SideMotorArray right_drive_motors(RobotHardware& hardware) {
  return {{
      &hardware.motor_fr1,
      &hardware.motor_fr2,
      &hardware.motor_br1,
      &hardware.motor_br2,
  }};
}

bool should_run_autonomous(vex::competition& competition) {
  return competition.isEnabled() && competition.isAutonomous();
}

double clamp_value(double value, double min_value, double max_value) {
  return std::min(std::max(value, min_value), max_value);
}

double clamp_unit_interval(double value) {
  return clamp_value(value, 0.0, 1.0);
}

double clamp_correction(double correction_pct, double max_abs_correction_pct) {
  return clamp_value(correction_pct, -max_abs_correction_pct, max_abs_correction_pct);
}

double smoothstep01(double value) {
  const double clamped = clamp_unit_interval(value);
  return clamped * clamped * (3.0 - 2.0 * clamped);
}

double planned_linear_speed_pct(
    double traveled_mm,
    double remaining_mm,
    double min_speed_pct,
    double max_speed_pct,
    double acceleration_window_mm,
    double deceleration_window_mm) {
  const double accel_ratio = acceleration_window_mm > 0.0
                                 ? smoothstep01(traveled_mm / acceleration_window_mm)
                                 : 1.0;
  const double decel_ratio = deceleration_window_mm > 0.0
                                 ? smoothstep01(remaining_mm / deceleration_window_mm)
                                 : 1.0;
  const double accel_cap_pct = min_speed_pct + (max_speed_pct - min_speed_pct) * accel_ratio;
  const double decel_cap_pct = min_speed_pct + (max_speed_pct - min_speed_pct) * decel_ratio;
  return std::min(accel_cap_pct, decel_cap_pct);
}

double drive_heading_correction_pct(double heading_error_degrees, double drive_speed_pct) {
  if (std::fabs(heading_error_degrees) <= kDriveHeadingDeadbandDegrees) {
    return 0.0;
  }

  const double scaled_max_correction_pct = std::min(
      kDriveHeadingCorrectionMaxPct,
      std::fabs(drive_speed_pct) * kDriveHeadingCorrectionSpeedRatio);
  return clamp_correction(
      heading_error_degrees * kDriveHeadingProportionalGain,
      scaled_max_correction_pct);
}

double deg_to_rad(double angle_deg) {
  return angle_deg * kPi / 180.0;
}

double rad_to_deg(double angle_rad) {
  return angle_rad * 180.0 / kPi;
}

double normalize_angle_deg(double angle_deg) {
  while (angle_deg > 180.0) {
    angle_deg -= 360.0;
  }
  while (angle_deg <= -180.0) {
    angle_deg += 360.0;
  }
  return angle_deg;
}

double blend_angle_deg(double start_deg, double end_deg, double alpha) {
  return normalize_angle_deg(
      start_deg + normalize_angle_deg(end_deg - start_deg) * clamp_unit_interval(alpha));
}

double heading_x_component(double heading_deg) {
  return std::sin(deg_to_rad(heading_deg));
}

double heading_y_component(double heading_deg) {
  return std::cos(deg_to_rad(heading_deg));
}

double heading_from_vector_deg(double x_mm, double y_mm) {
  return normalize_angle_deg(rad_to_deg(std::atan2(x_mm, y_mm)));
}

double average_revolutions(const SideMotorArray& motors) {
  double total = 0.0;
  for (vex::motor* motor : motors) {
    total += motor->position(vex::rev);
  }
  return total / static_cast<double>(motors.size());
}

DriveSideRevolutions sample_drive_side_revolutions(RobotHardware& hardware) {
  return {
      average_revolutions(left_drive_motors(hardware)),
      average_revolutions(right_drive_motors(hardware)),
  };
}

double center_delta_mm(
    const DriveSideRevolutions& previous_sample,
    const DriveSideRevolutions& current_sample) {
  const double left_delta_rev = current_sample.left_rev - previous_sample.left_rev;
  const double right_delta_rev = current_sample.right_rev - previous_sample.right_rev;
  return 0.5 * (left_delta_rev + right_delta_rev) * kMillimetersPerWheelRevolution;
}

void set_drive_power(RobotHardware& hardware, double left_pct, double right_pct) {
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

double local_inertial_heading_deg(RobotHardware& hardware) {
  return normalize_angle_deg(hardware.inertial.rotation(vex::deg));
}

double update_autonomous_pose_estimate(
    RobotHardware& hardware,
    RobotState& state,
    DriveSideRevolutions& previous_drive_sample) {
  const DriveSideRevolutions current_drive_sample = sample_drive_side_revolutions(hardware);
  const double previous_heading_deg = state.autonomous.estimated_heading_deg;
  const double current_heading_deg = local_inertial_heading_deg(hardware);
  const double delta_mm = center_delta_mm(previous_drive_sample, current_drive_sample);
  previous_drive_sample = current_drive_sample;

  if (std::fabs(delta_mm) > 1e-6) {
    const double travel_heading_deg = blend_angle_deg(previous_heading_deg, current_heading_deg, 0.5);
    state.autonomous.estimated_x_mm += delta_mm * heading_x_component(travel_heading_deg);
    state.autonomous.estimated_y_mm += delta_mm * heading_y_component(travel_heading_deg);
  }
  state.autonomous.estimated_heading_deg = current_heading_deg;

  return state.autonomous.estimated_heading_deg;
}

void refresh_autonomous_pose_estimate(RobotHardware& hardware, RobotState& state) {
  DriveSideRevolutions sample = sample_drive_side_revolutions(hardware);
  update_autonomous_pose_estimate(hardware, state, sample);
}

void reset_autonomous_frame(RobotHardware& hardware, RobotState& state) {
  hardware.inertial.resetRotation();
  state.autonomous = AutonomousState{};
  state.autonomous.initialized = true;
  state.autonomous.target_heading_deg = 0.0;
  state.autonomous.estimated_heading_deg = 0.0;
  state.autonomous.estimated_x_mm = 0.0;
  state.autonomous.estimated_y_mm = 0.0;
}

void ensure_autonomous_frame(RobotHardware& hardware, RobotState& state) {
  if (!state.autonomous.initialized) {
    reset_autonomous_frame(hardware, state);
  }
}

double laser_distance_timeout_ms(double distance_error_mm) {
  return kLaserDistanceBaseTimeoutMs +
         std::ceil(std::fabs(distance_error_mm) * static_cast<double>(kLaserDistanceTimeoutPerMm));
}

double go_to_pose_timeout_ms(double distance_mm, double heading_error_deg) {
  return kGoToPoseBaseTimeoutMs +
         std::ceil(std::fabs(distance_mm) * static_cast<double>(kGoToPoseTimeoutPerMm)) +
         std::ceil(std::fabs(heading_error_deg) * static_cast<double>(kGoToPoseTimeoutPerDegreeMs));
}

bool try_read_laser_distance_mm(RobotHardware& hardware, double& measured_distance_mm) {
  if (!hardware.laser_rangefinder.installed() || !hardware.laser_rangefinder.isObjectDetected()) {
    return false;
  }

  measured_distance_mm = hardware.laser_rangefinder.objectDistance(vex::distanceUnits::mm);
  return std::isfinite(measured_distance_mm) && measured_distance_mm > 0.0;
}

double turn_speed_pct(double error_degrees) {
  const double abs_error_degrees = std::fabs(error_degrees);
  const double min_speed_pct =
      abs_error_degrees > kTurnApproachWindowDegrees ? kTurnMinSpeedPct : kTurnApproachMinSpeedPct;
  return clamp_value(abs_error_degrees * kTurnProportionalGain, min_speed_pct, kTurnMaxSpeedPct);
}

double motion_sign_for_segment(
    TravelDirection travel_direction,
    double path_heading_deg,
    double current_heading_deg) {
  if (travel_direction == TravelDirection::kForward) {
    return 1.0;
  }
  if (travel_direction == TravelDirection::kReverse) {
    return -1.0;
  }

  const double forward_error_deg =
      std::fabs(normalize_angle_deg(path_heading_deg - current_heading_deg));
  const double reverse_error_deg =
      std::fabs(normalize_angle_deg(path_heading_deg + 180.0 - current_heading_deg));
  return reverse_error_deg + 5.0 < forward_error_deg ? -1.0 : 1.0;
}

void settle_after_motion() {
  vex::this_thread::sleep_for(kAutonomousSettleDelayMs);
}

void drive_distance_mm(
    RobotHardware& hardware,
    RobotState& state,
    vex::competition& competition,
    double distance_mm) {
  if (!should_run_autonomous(competition) || distance_mm == 0.0) {
    return;
  }

  ensure_autonomous_frame(hardware, state);
  refresh_autonomous_pose_estimate(hardware, state);

  const double travel_heading_deg = state.autonomous.target_heading_deg;
  const double target_x_mm =
      state.autonomous.estimated_x_mm + distance_mm * heading_x_component(travel_heading_deg);
  const double target_y_mm =
      state.autonomous.estimated_y_mm + distance_mm * heading_y_component(travel_heading_deg);
  const TravelDirection travel_direction =
      distance_mm >= 0.0 ? TravelDirection::kForward : TravelDirection::kReverse;
  go_to_pose(
      hardware,
      state,
      competition,
      target_x_mm,
      target_y_mm,
      travel_heading_deg,
      travel_direction);
}

void turn_deg(
    RobotHardware& hardware,
    RobotState& state,
    vex::competition& competition,
    double target_degrees) {
  if (!should_run_autonomous(competition) || target_degrees == 0.0) {
    return;
  }

  ensure_autonomous_frame(hardware, state);
  refresh_autonomous_pose_estimate(hardware, state);
  go_to_pose(
      hardware,
      state,
      competition,
      state.autonomous.estimated_x_mm,
      state.autonomous.estimated_y_mm,
      normalize_angle_deg(state.autonomous.target_heading_deg + target_degrees),
      TravelDirection::kAuto);
}

void run_overhang_motion(
    vex::motor& motor,
    double rotation_deg,
    double velocity_pct,
    bool wait_for_completion) {
  motor.setStopping(vex::hold);
  motor.spinFor(
      rotation_deg,
      vex::deg,
      velocity_pct,
      vex::velocityUnits::pct,
      wait_for_completion);
  if (wait_for_completion) {
    motor.stop(vex::hold);
  }
}

void wait_for_motor_motion(vex::motor& motor) {
  while (motor.isSpinning() && !motor.isDone()) {
    vex::this_thread::sleep_for(kAutonomousLoopDelayMs);
  }
  motor.stop(vex::hold);
}

void update_upper_overhang_mode(
    RobotHardware& hardware,
    RobotState& state,
    bool wait_for_completion = true) {
  OverhangMode& overhang_mode = state.overhang.upper_overhang_mode;
  if (overhang_mode == OverhangMode::Expansion) {
    overhang_mode = OverhangMode::Collapse;
    run_overhang_motion(hardware.upper_overhang_motor, 1100.0, 70.0, wait_for_completion);
  } else {
    overhang_mode = OverhangMode::Expansion;
    run_overhang_motion(hardware.upper_overhang_motor, -1100.0, 70.0, wait_for_completion);
  }
}

void update_middle_overhang_mode(
    RobotHardware& hardware,
    RobotState& state,
    bool wait_for_completion = true) {
  OverhangMode& overhang_mode = state.overhang.middle_overhang_mode;
  if (overhang_mode == OverhangMode::Expansion) {
    overhang_mode = OverhangMode::Collapse;
    run_overhang_motion(hardware.middle_overhang_motor, -300.0, 50.0, wait_for_completion);
  } else {
    overhang_mode = OverhangMode::Expansion;
    run_overhang_motion(hardware.middle_overhang_motor, 300.0, 50.0, wait_for_completion);
  }
}

void update_under_overhang_mode(
    RobotHardware& hardware,
    RobotState& state,
    bool wait_for_completion = true) {
  OverhangMode& overhang_mode = state.overhang.under_overhang_mode;
  if (overhang_mode == OverhangMode::Expansion) {
    overhang_mode = OverhangMode::Collapse;
    run_overhang_motion(hardware.under_overhang_motor, -250.0, 50.0, wait_for_completion);
  } else {
    overhang_mode = OverhangMode::Expansion;
    run_overhang_motion(hardware.under_overhang_motor, 250.0, 50.0, wait_for_completion);
  }
}

void update_mechanism_mode(RobotHardware& hardware, RobotState& state, int time_ms){
  for(int i=0;i<=time_ms;i+=kAutonomousLoopDelayMs){
    apply_mechanism_mode(hardware,state);
    vex::this_thread::sleep_for(kAutonomousLoopDelayMs);
  }
}

}  // namespace

void go_to_pose(
    RobotHardware& hardware,
    RobotState& state,
    vex::competition& competition,
    double target_x_mm,
    double target_y_mm,
    double target_heading_deg,
    TravelDirection travel_direction) {
  if (!should_run_autonomous(competition)) {
    return;
  }

  ensure_autonomous_frame(hardware, state);
  state.autonomous.target_heading_deg = normalize_angle_deg(target_heading_deg);

  DriveSideRevolutions drive_sample = sample_drive_side_revolutions(hardware);
  update_autonomous_pose_estimate(hardware, state, drive_sample);

  const double start_x_mm = state.autonomous.estimated_x_mm;
  const double start_y_mm = state.autonomous.estimated_y_mm;
  const double segment_dx_mm = target_x_mm - start_x_mm;
  const double segment_dy_mm = target_y_mm - start_y_mm;
  const double segment_length_mm = std::hypot(segment_dx_mm, segment_dy_mm);
  const double base_path_heading_deg = segment_length_mm > 1e-6
                                           ? heading_from_vector_deg(segment_dx_mm, segment_dy_mm)
                                           : state.autonomous.estimated_heading_deg;
  const double motion_sign =
      motion_sign_for_segment(travel_direction, base_path_heading_deg, state.autonomous.estimated_heading_deg);
  const int start_time_ms = hardware.brain.timer(vex::msec);
  const double timeout_ms = go_to_pose_timeout_ms(
      segment_length_mm,
      normalize_angle_deg(state.autonomous.target_heading_deg - state.autonomous.estimated_heading_deg));

  while (should_run_autonomous(competition)) {
    const int elapsed_ms = hardware.brain.timer(vex::msec) - start_time_ms;
    if (elapsed_ms >= timeout_ms) {
      break;
    }

    const double current_heading_deg = update_autonomous_pose_estimate(hardware, state, drive_sample);
    const double current_x_mm = state.autonomous.estimated_x_mm;
    const double current_y_mm = state.autonomous.estimated_y_mm;
    const double target_delta_x_mm = target_x_mm - current_x_mm;
    const double target_delta_y_mm = target_y_mm - current_y_mm;
    const double distance_to_target_mm = std::hypot(target_delta_x_mm, target_delta_y_mm);
    const double final_heading_error_deg =
        normalize_angle_deg(state.autonomous.target_heading_deg - current_heading_deg);

    if (distance_to_target_mm <= kGoToPosePositionToleranceMm &&
        std::fabs(final_heading_error_deg) <= kGoToPoseHeadingToleranceDegrees) {
      break;
    }

    double linear_speed_pct = 0.0;
    double desired_body_heading_deg = state.autonomous.target_heading_deg;

    if (segment_length_mm > 1e-6 && distance_to_target_mm > kGoToPosePositionToleranceMm) {
      const double segment_unit_x = segment_dx_mm / segment_length_mm;
      const double segment_unit_y = segment_dy_mm / segment_length_mm;
      const double along_track_mm = clamp_value(
          (current_x_mm - start_x_mm) * segment_unit_x +
              (current_y_mm - start_y_mm) * segment_unit_y,
          0.0,
          segment_length_mm);
      const double lookahead_mm = clamp_value(
          distance_to_target_mm * 0.6,
          kGoToPoseLookaheadMinMm,
          kGoToPoseLookaheadMaxMm);
      const double carrot_progress_mm = std::min(segment_length_mm, along_track_mm + lookahead_mm);
      const double carrot_x_mm = start_x_mm + segment_unit_x * carrot_progress_mm;
      const double carrot_y_mm = start_y_mm + segment_unit_y * carrot_progress_mm;
      const double carrot_delta_x_mm = carrot_x_mm - current_x_mm;
      const double carrot_delta_y_mm = carrot_y_mm - current_y_mm;

      desired_body_heading_deg =
          std::hypot(carrot_delta_x_mm, carrot_delta_y_mm) > 1e-6
              ? heading_from_vector_deg(carrot_delta_x_mm, carrot_delta_y_mm)
              : base_path_heading_deg;
      if (motion_sign < 0.0) {
        desired_body_heading_deg = normalize_angle_deg(desired_body_heading_deg + 180.0);
      }

      const double heading_blend_ratio = smoothstep01(
          (kGoToPoseHeadingBlendWindowMm -
           std::min(distance_to_target_mm, kGoToPoseHeadingBlendWindowMm)) /
          kGoToPoseHeadingBlendWindowMm);
      desired_body_heading_deg = blend_angle_deg(
          desired_body_heading_deg,
          state.autonomous.target_heading_deg,
          heading_blend_ratio);

      const double heading_error_deg =
          normalize_angle_deg(desired_body_heading_deg - current_heading_deg);
      const double heading_scale = clamp_unit_interval(
          (kGoToPoseHeadingSlowWindowDegrees - std::fabs(heading_error_deg)) /
          kGoToPoseHeadingSlowWindowDegrees);
      const double remaining_for_speed_mm = std::max(
          distance_to_target_mm,
          std::max(0.0, segment_length_mm - along_track_mm));
      const double planned_speed_pct = planned_linear_speed_pct(
          along_track_mm,
          remaining_for_speed_mm,
          kGoToPoseMinSpeedPct,
          kGoToPoseMaxSpeedPct,
          kGoToPoseAccelerationWindowMm,
          kGoToPoseDecelerationWindowMm);
      linear_speed_pct = heading_scale > 0.1
                             ? motion_sign * planned_speed_pct * (0.1 + 0.9 * heading_scale)
                             : 0.0;
    } else if (distance_to_target_mm > kGoToPosePositionToleranceMm) {
      const double path_heading_deg = heading_from_vector_deg(target_delta_x_mm, target_delta_y_mm);
      desired_body_heading_deg = motion_sign > 0.0
                                     ? path_heading_deg
                                     : normalize_angle_deg(path_heading_deg + 180.0);
      const double heading_error_deg =
          normalize_angle_deg(desired_body_heading_deg - current_heading_deg);
      const double heading_scale = clamp_unit_interval(
          (kGoToPoseHeadingSlowWindowDegrees - std::fabs(heading_error_deg)) /
          kGoToPoseHeadingSlowWindowDegrees);
      const double planned_speed_pct = planned_linear_speed_pct(
          0.0,
          distance_to_target_mm,
          kGoToPoseMinSpeedPct,
          kGoToPoseMaxSpeedPct,
          kGoToPoseAccelerationWindowMm,
          kGoToPoseDecelerationWindowMm);
      linear_speed_pct = heading_scale > 0.1
                             ? motion_sign * planned_speed_pct * (0.1 + 0.9 * heading_scale)
                             : 0.0;
    }

    double turn_command_pct = clamp_correction(
        normalize_angle_deg(desired_body_heading_deg - current_heading_deg) *
            kGoToPoseTurnProportionalGain,
        kGoToPoseTurnMaxPct);
    if (distance_to_target_mm <= kGoToPosePositionToleranceMm) {
      const double rotate_speed_pct = turn_speed_pct(final_heading_error_deg);
      turn_command_pct = final_heading_error_deg >= 0.0 ? rotate_speed_pct : -rotate_speed_pct;
      linear_speed_pct = 0.0;
      if (std::fabs(final_heading_error_deg) <= kTurnToleranceDegrees) {
        break;
      }
    }

    set_drive_power(
        hardware,
        linear_speed_pct + 0.5 * turn_command_pct,
        linear_speed_pct - 0.5 * turn_command_pct);
    vex::this_thread::sleep_for(kAutonomousLoopDelayMs);
  }

  stop_drive(hardware, vex::hold);
  update_autonomous_pose_estimate(hardware, state, drive_sample);
  settle_after_motion();
}

void drive_to_laser_distance_mm(
    RobotHardware& hardware,
    RobotState& state,
    vex::competition& competition,
    double target_distance_mm,
    double max_speed_pct) {
  if (!should_run_autonomous(competition) || target_distance_mm <= 0.0) {
    return;
  }

  ensure_autonomous_frame(hardware, state);
  const double laser_max_speed_pct =
      max_speed_pct > 0.0 ? max_speed_pct : kLaserDistanceMaxSpeedPct;

  double measured_distance_mm = 0.0;
  if (!try_read_laser_distance_mm(hardware, measured_distance_mm)) {
    stop_drive(hardware, vex::hold);
    return;
  }

  DriveSideRevolutions drive_sample = sample_drive_side_revolutions(hardware);
  update_autonomous_pose_estimate(hardware, state, drive_sample);
  const int motion_start_ms = hardware.brain.timer(vex::msec);
  const double initial_distance_error_mm = std::fabs(measured_distance_mm - target_distance_mm);
  const double timeout_ms = laser_distance_timeout_ms(measured_distance_mm - target_distance_mm);

  while (should_run_autonomous(competition)) {
    const int elapsed_ms = hardware.brain.timer(vex::msec) - motion_start_ms;
    if (elapsed_ms >= timeout_ms) {
      break;
    }

    if (!try_read_laser_distance_mm(hardware, measured_distance_mm)) {
      break;
    }

    const double current_heading_degrees =
        update_autonomous_pose_estimate(hardware, state, drive_sample);
    const double distance_error_mm = measured_distance_mm - target_distance_mm;
    if (std::fabs(distance_error_mm) <= kLaserDistanceToleranceMm) {
      break;
    }

    const double traveled_toward_target_mm =
        std::max(0.0, initial_distance_error_mm - std::fabs(distance_error_mm));
    const double drive_direction = distance_error_mm > 0.0 ? 1.0 : -1.0;
    const double commanded_speed_pct = drive_direction * planned_linear_speed_pct(
        traveled_toward_target_mm,
        std::fabs(distance_error_mm),
        kLaserDistanceMinSpeedPct,
        laser_max_speed_pct,
        kLaserDistanceAccelerationWindowMm,
        kLaserDistanceDecelerationWindowMm);
    const double heading_error_degrees = normalize_angle_deg(
        state.autonomous.target_heading_deg - current_heading_degrees);
    const double heading_correction_pct =
        drive_heading_correction_pct(heading_error_degrees, commanded_speed_pct);
    set_drive_power(
        hardware,
        commanded_speed_pct + heading_correction_pct,
        commanded_speed_pct - heading_correction_pct);
    vex::this_thread::sleep_for(kAutonomousLoopDelayMs);
  }

  stop_drive(hardware, vex::hold);
  update_autonomous_pose_estimate(hardware, state, drive_sample);
  settle_after_motion();
}

void run_routine(RobotHardware& hardware, RobotState& state, vex::competition& competition) {

  state.chassis.stop_brake_type = vex::hold;
  reset_autonomous_frame(hardware, state);
  //hardware.calibrate_inertial_sensor();
  //hardware.show_calibrated();

  //update_under_overhang_mode(hardware,state);
  update_middle_overhang_mode(hardware,state,false);
  update_upper_overhang_mode(hardware,state,false);
  
  drive_to_laser_distance_mm(hardware,state,competition,525.0);
  //drive_distance_mm(hardware, state, competition, kFirstDriveDistanceMm);
  turn_deg(hardware, state, competition, -90.0);
  update_under_overhang_mode(hardware,state,true);

  update_intake_mode(hardware,state);
  drive_to_laser_distance_mm(hardware, state, competition, 135.0, 30.0);
  update_mechanism_mode(hardware,state,5000);
  //vex::this_thread::sleep_for(5000);
  update_intake_mode(hardware,state);

  drive_distance_mm(hardware, state, competition, -320.0);
  update_under_overhang_mode(hardware,state,false);
  turn_deg(hardware, state, competition, 90.0);
  //drive_distance_mm(hardware, state, competition, -377.0);
  drive_to_laser_distance_mm(hardware, state, competition, 510.0);
  turn_deg(hardware, state, competition, 90.0);
  drive_distance_mm(hardware, state, competition, 502.0);
  

  update_upperthrow_mode(hardware,state);
  vex::this_thread::sleep_for(5000);
  update_upperthrow_mode(hardware,state);

  // drive_distance_mm(hardware, state, competition, -397.5);
  drive_distance_mm(hardware, state, competition, -200.0);
  turn_deg(hardware, state, competition, -90.0);
  drive_distance_mm(hardware, state, competition, -400.0);
  drive_to_laser_distance_mm(hardware, state, competition, 820.0);
  turn_deg(hardware, state, competition, 90.0);
  drive_distance_mm(hardware, state, competition, 600.0);
  drive_distance_mm(hardware, state, competition, -577.5);
  turn_deg(hardware, state, competition, 45.0);
  update_under_overhang_mode(hardware,state,false);
  drive_distance_mm(hardware, state, competition, 900.0);
  wait_for_motor_motion(hardware.under_overhang_motor);

  update_middlethrow_mode(hardware,state);
  vex::this_thread::sleep_for(5000);
  update_middlethrow_mode(hardware,state);

  stop_drive(hardware, vex::hold);
}

}  // namespace basic::hardware::robots::autonomous
