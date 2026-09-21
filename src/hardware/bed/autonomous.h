#ifndef BASIC_SRC_HARDWARE_BED_AUTONOMOUS_H_
#define BASIC_SRC_HARDWARE_BED_AUTONOMOUS_H_

#include "hardware/bed/robot_hardware.h"
#include "hardware/bed/robot_state.h"
#include "mechanism/dual_pneumatic.h"
#include "mechanism/pneumatic_gripper.h"

namespace basic::hardware::bed::autonomous {

/// 机构动作默认超时：超时后停机并返回，避免自走流程被卡死的机构拖住
inline constexpr int kLiftMoveTimeoutMs = 6000;
inline constexpr int kArmMoveTimeoutMs = 10000;

/// 抬升目标位置范围：linear_lift_set_position 的目标是两电机位置之和
inline constexpr double kLiftPositionMin = 0.0;
inline constexpr double kLiftPositionMax = 2.0 * kLiftMaxDeg;

/// 自走运行条件：比赛使能且处于 autonomous 阶段（与其它机器人一致）
bool should_run_autonomous(vex::competition& competition);

/// 自走开始前的安全准备：IMU 归零、清空自走/航向状态、停全部输出
void prepare_autonomous(RobotHardware& hardware, RobotState& state);

/// 自走阶段内等待（中途退出 autonomous 会提前返回）
void wait_ms(vex::competition& competition, int duration_ms);

/// IMU 当前航向（deg，[-180, 180)）
double current_heading_deg(RobotHardware& hardware);

/// 底盘直通速度（pct）：forward = 前后、strafe = 平移、turn = 自转
/// 直接给到四轮，跳过摇杆死区/灵敏度/曲线，命令值完全确定，适合自走
void set_body_velocity(RobotHardware& hardware,
                       double forward_pct,
                       double strafe_pct,
                       double turn_pct,
                       vex::brakeType brake_type = vex::hold);

/// 定时动作：pct 恒定，到时停。
/// hold_heading = true 时用 IMU 锁进入本函数时的航向（适用于前进/平移）
void drive_timed(RobotHardware& hardware,
                 vex::competition& competition,
                 double forward_pct,
                 double strafe_pct,
                 double turn_pct,
                 int duration_ms,
                 bool hold_heading = false,
                 vex::brakeType brake_type = vex::hold);

/// 定时前进（speed_pct > 0 前进，< 0 后退），默认锁航向
void drive_forward_timed(RobotHardware& hardware,
                         vex::competition& competition,
                         double speed_pct,
                         int duration_ms,
                         bool hold_heading = true);

/// 定时平移（符号约定与 bed 手动平移通道一致），默认锁航向
void strafe_timed(RobotHardware& hardware,
                  vex::competition& competition,
                  double speed_pct,
                  int duration_ms,
                  bool hold_heading = true);

/// IMU 闭环转向到目标航向（deg）：比例控制 + 最小速度 + 近目标降速
void turn_to_heading_deg(RobotHardware& hardware,
                         vex::competition& competition,
                         double target_heading_deg,
                         double max_turn_speed_pct = 30.0,
                         vex::brakeType brake_type = vex::hold);

/// 机构工具：吸球启停 / 夹爪抓握 / 框住机构
void run_intake(RobotHardware& hardware, bool running);
void set_gripper_grasp(RobotHardware& hardware, bool grasp);
void set_frame_engaged(RobotHardware& hardware, bool engaged);

/// 抬升到目标位置（两电机位置之和，0 ~ 2 * kLiftMaxDeg），阻塞直到到位/超时/退出自走
void lift_move_to(RobotHardware& hardware,
                  vex::competition& competition,
                  double target_position,
                  int timeout_ms = kLiftMoveTimeoutMs);

/// 抬升到行程比例（0.0 = 下限，1.0 = 上限）
void lift_move_to_fraction(RobotHardware& hardware,
                           vex::competition& competition,
                           double fraction,
                           int timeout_ms = kLiftMoveTimeoutMs);

/// 整臂展开/收缩（堵转找限位按序动作），返回是否在超时前完成。
/// 展开前需先 set_frame_engaged(hardware, true)（外部联锁与手动循环一致）；
/// 本函数在等待期间会周期调用 arm_2dof_update 推进序列。
bool set_arm_extended(RobotHardware& hardware,
                      vex::competition& competition,
                      bool extended,
                      int timeout_ms = kArmMoveTimeoutMs);

/// 统一停机：底盘 + 全部机构（与 bed 手动退出路径一致）
void stop_all_outputs(RobotHardware& hardware,
                      RobotState& state,
                      vex::brakeType drive_brake_type = vex::coast);

void run_routine(
    basic::hardware::bed::RobotHardware& hardware,
    basic::hardware::bed::RobotState& state,
    vex::competition& competition);

}  // namespace basic::hardware::bed::autonomous

#endif  // BASIC_SRC_HARDWARE_BED_AUTONOMOUS_H_
