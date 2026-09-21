#include "hardware/bed/autonomous.h"

#include "chassis/bed_chassis.h"
#include "chassis/x_drive.h"
#include "control/motor_control.h"
#include "mechanism/arm_2dof.h"
#include "mechanism/dual_pneumatic.h"
#include "mechanism/intake.h"
#include "mechanism/linear_lift.h"
#include "mechanism/pneumatic_gripper.h"

#include <algorithm>
#include <cmath>

namespace basic::hardware::bed::autonomous {

namespace {

using basic::hardware::bed::RobotHardware;
using basic::hardware::bed::RobotState;

/// 自走循环节拍（ms）：与 driver/手动循环一致
inline constexpr int kLoopDelayMs = 10;

/// 定时前进/平移的航向保持（比例控制，输出限幅 pct）
inline constexpr double kDriveHeadingGain = 0.5;
inline constexpr double kDriveHeadingCorrectionMaxPct = 10.0;

/// IMU 闭环转向参数（参考 second_robot：比例 + 最小速度 + 近目标降速）
inline constexpr double kTurnToleranceDeg = 2.0;
inline constexpr double kTurnGain = 0.6;
inline constexpr double kTurnMinSpeedPct = 8.0;
inline constexpr double kTurnApproachMinSpeedPct = 4.0;
inline constexpr double kTurnApproachWindowDeg = 12.0;
inline constexpr int kTurnBaseTimeoutMs = 700;
inline constexpr int kTurnTimeoutPerDegMs = 10;

double clamp_value(double value, double min_value, double max_value) {
  return std::min(std::max(value, min_value), max_value);
}

double clamp_pct(double value) {
  return clamp_value(value, -100.0, 100.0);
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

/// 四轮直通下发（pct）：固件速度环模式，命令值完全确定
void set_wheel_velocity(
    RobotHardware& hardware,
    double fl_pct,
    double fr_pct,
    double bl_pct,
    double br_pct,
    vex::brakeType brake_type) {
  basic::chassis::x_drive_set_output(
      hardware.bed_chassis,
      clamp_pct(fl_pct),
      clamp_pct(fr_pct),
      clamp_pct(bl_pct),
      clamp_pct(br_pct),
      brake_type);
}

}  // namespace

bool should_run_autonomous(vex::competition& competition) {
  return competition.isEnabled() && competition.isAutonomous();
}

double current_heading_deg(RobotHardware& hardware) {
  if (!hardware.imu.installed()) {
    return 0.0;
  }
  return normalize_angle_deg(hardware.imu.rotation(vex::deg));
}

void set_body_velocity(
    RobotHardware& hardware,
    double forward_pct,
    double strafe_pct,
    double turn_pct,
    vex::brakeType brake_type) {
  // 混合公式与 x_drive_update 一致：
  //   fl = forward + strafe + turn    fr = forward - strafe - turn
  //   bl = forward - strafe + turn    br = forward + strafe - turn
  const double fl = forward_pct + strafe_pct + turn_pct;
  const double fr = forward_pct - strafe_pct - turn_pct;
  const double bl = forward_pct - strafe_pct + turn_pct;
  const double br = forward_pct + strafe_pct - turn_pct;
  set_wheel_velocity(hardware, fl, fr, bl, br, brake_type);
}

void stop_all_outputs(
    RobotHardware& hardware,
    RobotState& state,
    vex::brakeType drive_brake_type) {
  state.controller = basic::hardware::shared::ControllerInputState{};
  basic::chassis::bed_chassis_stop(hardware.bed_chassis, drive_brake_type);
  basic::mechanism::intake_stop(hardware.intake, vex::coast);
  basic::mechanism::pneumatic_gripper_stop(hardware.pneumatic_gripper);
  basic::mechanism::arm_2dof_stop(hardware.arm_2dof, vex::hold);
  basic::mechanism::linear_lift_stop(hardware.lift, vex::hold);
  basic::mechanism::dual_pneumatic_stop(hardware.dual_pneumatic);
}

void prepare_autonomous(RobotHardware& hardware, RobotState& state) {
  if (hardware.imu.installed()) {
    hardware.imu.resetRotation();
  }

  state.autonomous = basic::hardware::shared::AutonomousState{};
  state.autonomous.initialized = true;

  // 航向保持属于手动路径：自走期间不使用，进入时清掉历史状态
  hardware.yaw_hold.state = basic::chassis::YawHoldState{};
  hardware.yaw_hold.pid.reset();
  hardware.yaw_hold.last_ms = 0;

  basic::chassis::bed_chassis_state(hardware.bed_chassis).stop_brake_type = vex::hold;
  stop_all_outputs(hardware, state, vex::hold);
}

void wait_ms(vex::competition& competition, int duration_ms) {
  int elapsed_ms = 0;
  while (elapsed_ms < duration_ms && should_run_autonomous(competition)) {
    const int step_ms = std::min(kLoopDelayMs, duration_ms - elapsed_ms);
    vex::this_thread::sleep_for(step_ms);
    elapsed_ms += step_ms;
  }
}

void drive_timed(
    RobotHardware& hardware,
    vex::competition& competition,
    double forward_pct,
    double strafe_pct,
    double turn_pct,
    int duration_ms,
    bool hold_heading,
    vex::brakeType brake_type) {
  if (!should_run_autonomous(competition) || duration_ms <= 0) {
    return;
  }

  const double target_heading_deg = current_heading_deg(hardware);
  int elapsed_ms = 0;
  while (elapsed_ms < duration_ms && should_run_autonomous(competition)) {
    double command_turn_pct = turn_pct;
    if (hold_heading) {
      const double heading_error_deg =
          normalize_angle_deg(target_heading_deg - current_heading_deg(hardware));
      command_turn_pct += clamp_value(
          heading_error_deg * kDriveHeadingGain,
          -kDriveHeadingCorrectionMaxPct,
          kDriveHeadingCorrectionMaxPct);
    }

    set_body_velocity(
        hardware, forward_pct, strafe_pct, command_turn_pct, brake_type);
    vex::this_thread::sleep_for(kLoopDelayMs);
    elapsed_ms += kLoopDelayMs;
  }

  set_body_velocity(hardware, 0.0, 0.0, 0.0, brake_type);
}

void drive_forward_timed(
    RobotHardware& hardware,
    vex::competition& competition,
    double speed_pct,
    int duration_ms,
    bool hold_heading) {
  drive_timed(
      hardware,
      competition,
      speed_pct,
      0.0,
      0.0,
      duration_ms,
      hold_heading,
      vex::hold);
}

void strafe_timed(
    RobotHardware& hardware,
    vex::competition& competition,
    double speed_pct,
    int duration_ms,
    bool hold_heading) {
  drive_timed(
      hardware,
      competition,
      0.0,
      speed_pct,
      0.0,
      duration_ms,
      hold_heading,
      vex::hold);
}

void turn_to_heading_deg(
    RobotHardware& hardware,
    vex::competition& competition,
    double target_heading_deg,
    double max_turn_speed_pct,
    vex::brakeType brake_type) {
  if (!should_run_autonomous(competition) || !hardware.imu.installed()) {
    return;
  }

  const double normalized_target_deg = normalize_angle_deg(target_heading_deg);
  const double initial_error_deg =
      normalize_angle_deg(normalized_target_deg - current_heading_deg(hardware));
  const int timeout_ms =
      kTurnBaseTimeoutMs +
      static_cast<int>(std::ceil(
          std::fabs(initial_error_deg) * static_cast<double>(kTurnTimeoutPerDegMs)));

  int elapsed_ms = 0;
  while (elapsed_ms < timeout_ms && should_run_autonomous(competition)) {
    const double error_deg =
        normalize_angle_deg(normalized_target_deg - current_heading_deg(hardware));
    if (std::fabs(error_deg) <= kTurnToleranceDeg) {
      break;
    }

    const double abs_error_deg = std::fabs(error_deg);
    const double min_speed_pct = abs_error_deg <= kTurnApproachWindowDeg
                                     ? kTurnApproachMinSpeedPct
                                     : kTurnMinSpeedPct;
    const double speed_pct =
        clamp_value(abs_error_deg * kTurnGain, min_speed_pct, max_turn_speed_pct);
    const double turn_pct = error_deg >= 0.0 ? speed_pct : -speed_pct;

    set_body_velocity(hardware, 0.0, 0.0, turn_pct, brake_type);
    vex::this_thread::sleep_for(kLoopDelayMs);
    elapsed_ms += kLoopDelayMs;
  }

  set_body_velocity(hardware, 0.0, 0.0, 0.0, brake_type);
}

void run_intake(RobotHardware& hardware, bool running) {
  basic::mechanism::intake_set_running(hardware.intake, running);
}

void set_gripper_grasp(RobotHardware& hardware, bool grasp) {
  basic::mechanism::pneumatic_gripper_set_state(
      hardware.pneumatic_gripper,
      grasp ? basic::mechanism::PneumaticGripperMode::kGrasp
            : basic::mechanism::PneumaticGripperMode::kRelease);
}

void set_frame_engaged(RobotHardware& hardware, bool engaged) {
  basic::mechanism::dual_pneumatic_set_mode(
      hardware.dual_pneumatic,
      engaged ? basic::mechanism::DualPneumaticMode::kEngaged
              : basic::mechanism::DualPneumaticMode::kReleased);
}

void lift_move_to(
    RobotHardware& hardware,
    vex::competition& competition,
    double target_position,
    int timeout_ms) {
  if (!should_run_autonomous(competition)) {
    return;
  }

  const double target =
      clamp_value(target_position, kLiftPositionMin, kLiftPositionMax);
  basic::mechanism::linear_lift_set_position(hardware.lift, target);

  // 等一个周期让 spinToPosition 生效，避免 isDone 误判为已到位
  vex::this_thread::sleep_for(kLoopDelayMs);

  int elapsed_ms = kLoopDelayMs;
  bool done = false;
  while (elapsed_ms < timeout_ms && should_run_autonomous(competition)) {
    if (basic::control::get_done(hardware.lift.lift_motor1()) &&
        basic::control::get_done(hardware.lift.lift_motor2())) {
      done = true;
      break;
    }
    vex::this_thread::sleep_for(kLoopDelayMs);
    elapsed_ms += kLoopDelayMs;
  }

  if (!done) {
    basic::mechanism::linear_lift_stop(hardware.lift, vex::hold);
  }
}

void lift_move_to_fraction(
    RobotHardware& hardware,
    vex::competition& competition,
    double fraction,
    int timeout_ms) {
  const double clamped_fraction = clamp_value(fraction, 0.0, 1.0);
  const double target =
      kLiftPositionMin + clamped_fraction * (kLiftPositionMax - kLiftPositionMin);
  lift_move_to(hardware, competition, target, timeout_ms);
}

bool set_arm_extended(
    RobotHardware& hardware,
    vex::competition& competition,
    bool extended,
    int timeout_ms) {
  if (!should_run_autonomous(competition)) {
    return false;
  }

  basic::mechanism::arm_2dof_set_extended(hardware.arm_2dof, extended);

  int elapsed_ms = 0;
  while (elapsed_ms < timeout_ms && should_run_autonomous(competition)) {
    // 展开许可跟随框住机构（与 bed 手动循环的联锁一致）
    basic::mechanism::arm_2dof_set_extend_allowed(
        hardware.arm_2dof,
        basic::mechanism::dual_pneumatic_state(hardware.dual_pneumatic).mode ==
            basic::mechanism::DualPneumaticMode::kEngaged);
    basic::mechanism::arm_2dof_update(
        hardware.arm_2dof, basic::mechanism::Arm2DofCommand{});

    const auto& arm_state = basic::mechanism::arm_2dof_state(hardware.arm_2dof);
    if (arm_state.arm_sequence_step == 0 && arm_state.arm_extended == extended) {
      return true;
    }

    vex::this_thread::sleep_for(kLoopDelayMs);
    elapsed_ms += kLoopDelayMs;
  }

  basic::mechanism::arm_2dof_stop(hardware.arm_2dof, vex::hold);
  return false;
}

void run_routine(
    basic::hardware::bed::RobotHardware& hardware,
    basic::hardware::bed::RobotState& state,
    vex::competition& competition) {
  if (!should_run_autonomous(competition)) {
    return;
  }

  prepare_autonomous(hardware, state);

  // TODO(bed): 用上面的工具函数编排自走流程，示例：
  //   drive_forward_timed(hardware, competition, 30.0, 800);
  //   turn_to_heading_deg(hardware, competition, 90.0);
  //   lift_move_to_fraction(hardware, competition, 1.0);
  //   set_frame_engaged(hardware, true);
  //   set_arm_extended(hardware, competition, true);
  //   run_intake(hardware, true);

  stop_all_outputs(hardware, state, vex::hold);
}

}  // namespace basic::hardware::bed::autonomous
