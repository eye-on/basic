#ifndef BASIC_SRC_HARDWARE_BED_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_BED_ROBOT_HARDWARE_H_

#include "chassis/bed_chassis.h"
#include "chassis/yaw_hold.h"
#include "mechanism/arm_2dof.h"
#include "mechanism/dual_pneumatic.h"
#include "mechanism/intake.h"
#include "mechanism/linear_lift.h"
#include "mechanism/pneumatic_gripper.h"
#include "vex.h"

namespace basic::hardware::bed {

inline constexpr int kRefreshTime = 10;

// 固定速度测试模式（L2 循环切换）：0 = 手动跟摇杆，其余 = 四轮固定 pct（前进方向）
// 第一项必须是 0（手动）；用途：空载/半加载/落地用同一组确定命令对比测量
inline constexpr double kTestSpeedPct[] = {0.0, 30.0, 50.0, 100.0};
inline constexpr int kTestSpeedCount = 4;

// 固定速度测试模式下是否也启用 IMU 航向保持：
//   true （当前）= 测试时也锁航向（"一直闭环"）；R2 可临时关掉
//   false         = 纯速控基线，用于测量轮速波动/跑偏本身（做 A/B 对比时改这里）
inline constexpr bool kYawHoldInTestMode = true;

// 调试打印内容：
//   true （当前）= **只打印手柄摇杆行**（A 行，100Hz：四轴原始值 + 四轮下发 pct×10）
//   false         = 完整诊断流（D 100Hz + A 25Hz + C/Y/K 10Hz）
inline constexpr bool kDebugAxisOnly = true;

// 摇杆零位吸附（pct）：松手时手柄 ADC 有 ±1 量化抖动，|轴值| ≤ 此值按 0 处理
// 底盘死区本来就能挡住 ±1，但航向保持会把 axis4 的抖动积分进目标 yaw（turn_deadzone=0 时约 1.2°/s 漂移）
inline constexpr int kAxisSnapPct = 1;

// 抬升限位：下限 = 电机起始位置（上电编码器为 0），上限 = 起始位置 + 行程增量
inline constexpr double kLiftStartDeg = 0.0;    // 电机起始位置（默认 0）
inline constexpr double kLiftTravelDeg = 3500.0;  // 行程增量（上限 = 起始 + 3500°）
inline constexpr double kLiftMaxDeg = kLiftStartDeg + kLiftTravelDeg;  // 上限

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  // IMU：空闲智能口 2 / 12 / 15 / 16 任选（底盘占 1,3,4,5,17,18,19,20；
  // 机构占 6,7,8,9,10,11,13,14）。未插 IMU 时航向保持自动失效（不介入）。
  vex::inertial imu{vex::PORT12};
  basic::chassis::YawHold yaw_hold;
  basic::chassis::BedChassis bed_chassis;
  basic::mechanism::Intake intake;
  basic::mechanism::PneumaticGripper pneumatic_gripper;
  basic::mechanism::Arm2Dof arm_2dof;
  basic::mechanism::LinearLift lift;
  basic::mechanism::DualPneumatic dual_pneumatic;

  // 实际接线（2026-09-05）：右后 BR=1/2、右前 FR=5/6、左后 BL=7/8、左前 FL=9/19。
  // 底盘接线（最新）：左前 FL=4/5、右前 FR=17/18、左后 BL=1/3、右后 BR=19/20。
  // 每轮 2 电机驱动同一个轮：同轴同向安装则两个电机 reversed 相同；
  // 对角轮方向互为镜像（X 底盘惯例：FL/BL 与 FR/BR 相反）——TODO: 实测后核对。
  // 底盘控制方式（当前）：简单速控 —— 摇杆 → 死区/灵敏度 → X 混合 → 曲线整形
  //   → 直接作为电机固件速度环的目标速度下发（无软件速度环、无加减速斜率限制）
  // 备选（其他机器人沿用）：力控（电压直驱，±100 pct = ±12000 mV，扭矩 ∝ 电压）
  //   静摩擦补偿 2 pct / 输出死区 1.0 pct 见 arcade_drive.h
  // 调试键位：L1 = 打印开关（100Hz CSV：D 行 = 同轮两电机 rpm×4 组 + 相位×4 + 饱和，
  //                                   C 行 10Hz = 同轮两电机电流×4 组，
  //                                   Y 行 10Hz = yaw/目标/误差/修正 等航向量）；
  //           L2 = 固定速度测试模式循环（手动 → 30% → 50% → 100% → 手动）
  //           R1 = 航向目标重锚定（把当前车头方向设为锁定目标）；
  //           R2 = 航向闭环开关（临时关掉做对比）
  // 键位：A = 吸球 6/7/8 启停；Y = 机械臂整臂 收缩↔展开（13/10 + 14，自动按序）；
  //       上/下 = 抬升 9/11 升/降（限位 0~3500°）；X = 夹爪（ADI B）；B = 框住（ADI A/C）。
  // 联锁：框住机构处于展开(松开)态时禁止机械臂展开（收回始终允许）——
  //       即开机默认（框住=展开）无法展开机械臂，需先按 B 让框住进入框住态。
  RobotHardware()
      // IMU 航向保持：目标 yaw = 底盘**整形后 turn 指令**的时域积分；IMU 闭环修正 turn 通道
      //   误差 deg → 修正 pct：kp / ki / 输出与积分限幅 ±20（参数见下，已实测调过）
      //   运行时：R1 = 把当前车头方向重设为锁定目标；R2 = 开关航向闭环
      //   ⚠ 当前状态：**暂时关闭**（enabled = false）→ 航向修正在第一行就返回 0，
      //     既不积分也不修正，行为等同纯速控。需要时按 R2 临时打开（重启恢复关闭），
      //     或把下面的 enabled 改回 true。
      : yaw_hold(basic::chassis::yaw_hold_init({
            {0.5, 0.00, 0.0, -20.0, 20.0, -8.0, 8.0, 0.4},  // pid: kp,ki,kd,out±,i±,deadzone
            0,       // turn_deadzone：整形后 turn 指令的死区（pct，底盘已做死区，这里通常 0）
            5.6,     // deg_per_sec_per_turn_pct：标定值（1 pct turn 指令 ≈ 5.6°/s）
            20.0,    // max_correction_pct：修正限幅
            12.0,    // max_turn_lead_deg：打杆期间允许的目标超前量（防积分跑飞）
            0.0,     // rate_damping_pct_per_dps：IMU 角速度阻尼（0 = 关，先用 P）
            true,    // reanchor_on_release：松杆瞬间接受当前车头，消除"松手后又转几度"
            1,       // sign：方向修正（±1）
            false,   // enabled：暂时关闭（true = 一直实时闭环；运行时可 R2 切换）
        })),
        bed_chassis(basic::chassis::bed_chassis_init({
            {{ // 左前 FL：4, 5
                {vex::PORT4, vex::ratio6_1, true},   // TODO: 方向核对
                {vex::PORT5, vex::ratio6_1, true},
            }},
            {{ // 右前 FR：17, 18
                {vex::PORT17, vex::ratio6_1, false},  // TODO: 方向核对
                {vex::PORT18, vex::ratio6_1, false},
            }},
            {{ // 左后 BL：1, 3
                {vex::PORT1, vex::ratio6_1, true},   // TODO: 方向核对
                {vex::PORT3, vex::ratio6_1, true},
            }},
            {{ // 右后 BR：19, 20
                {vex::PORT19, vex::ratio6_1, false},  // TODO: 方向核对
                {vex::PORT20, vex::ratio6_1, false},
            }},
            10,   // deadzone（默认三轴死区）
            1.0,  // forward_sensitivity
            1.0,  // strafe_sensitivity
            0.5,  // turn_sensitivity
            2,    // deadzone_forward：0 = 用默认 10（轴 2，前后）
            2,   // deadzone_strafe：轴 1（左摇杆左右 / 平移）死区 30
            2,    // deadzone_turn：0 = 用默认 10（轴 4，旋转）
            // use_firmware_velocity：pct 直接作为电机固件速度环的目标速度下发
            // （简单速控：无软件速度环、无加减速斜率限制，摇杆 → 速度直通）
            true,
        })),
        // 吸球模块：3 电机（6/7/8）开环定速，A 键边沿启停
        intake(basic::mechanism::intake_init({
            {vex::PORT6, vex::ratio6_1, false},   // 电机 A
            {vex::PORT7, vex::ratio6_1, false},   // 电机 B
            {vex::PORT8, vex::ratio6_1, true},  // 电机 C
            75.0,                                // intake_speed_pct（开环）
        })),
        // 气缸夹爪：抓握/松开两态，X 边沿切换；电磁阀接 ADI B
        // （inverted：抓握对应高电平=false）；上电初始 = 展开（张开/松开）
        pneumatic_gripper(basic::mechanism::pneumatic_gripper_init({
            {brain.ThreeWirePort.B},  // 气缸电磁阀三线口
            false,                    // inverted
            basic::mechanism::PneumaticGripperMode::kRelease,  // 上电初始：展开（张开）
        })),
        // 机械臂电机 13/10（关节1 双电机）、14（关节2）；
        // 控制方式：开环定速 + 堵转找限位（两个位置都有机械硬限位）
        //   Y 单键边沿：整臂 收缩 ↔ 展开（自动按序：展开 关1→关2；收回 关2→关1）
        // 联锁：关节1 未顶到展开限位时关节2 禁止展开；关节2 未收回时关节1 禁止收回
        // 上电假定两关节都在收回位（home）；alt_sign 实测反了改 -1
        arm_2dof(basic::mechanism::arm_2dof_init({
            {vex::PORT13, vex::ratio36_1, true},   // 关节 1 电机组 A
            {vex::PORT10, vex::ratio36_1, false},  // 关节 1 电机组 B
            {vex::PORT14, vex::ratio18_1, false},  // 关节 2
            basic::mechanism::Arm2DofMode::kStallLimit,  // 开环 + 堵转找限位
            30,      // velocity_speed_pct（开环模式备用）
            vex::deg,
            30.0,    // position_speed_pct（编码器位置环模式备用）
            vex::hold,  // stop_brake_type
            0.0,     // joint1_home（占位，堵转模式不使用）
            220.0,   // joint1_alt（占位，堵转模式不使用）
            0.0,     // joint2_home（占位，堵转模式不使用）
            200.0,   // joint2_alt（占位，堵转模式不使用）
            true,    // joint12_interlock：联锁开启
            1,       // joint1_alt_sign：+1 = 正向朝展开限位（实测反了改 -1）
            1,       // joint2_alt_sign：+1 = 正向朝展开限位（实测反了改 -1）
            30.0,    // stall_speed_pct：顶限位时的开环速度
            5.0,     // stall_velocity_rpm：堵转判定阈值
            100,     // stall_grace_ms：起动宽限（此时间内不检测）
            100,     // stall_confirm_ms：连续低于阈值多久判到位
            4000,    // stall_timeout_ms：单次行程安全超时
        })),
        // 抬升机构：双电机 9/11，上 升 / 下 降（边沿切换）；
        // 限位：编码器限位（0 ~ 3500°）+ 堵转停止（双电机低速判顶，锁定该方向）
        // TODO: 行程增量与齿比/方向按实际标定
        lift(basic::mechanism::linear_lift_init({
            {{vex::PORT9, vex::ratio18_1, true}, kLiftStartDeg, kLiftMaxDeg},   // 电机 1
            {{vex::PORT11, vex::ratio18_1, false}, kLiftStartDeg, kLiftMaxDeg},  // 电机 2
            70.0,   // closed_loop_speed_pct
            70.0,   // open_loop_speed_pct
            50.0,   // closed_loop_speed_down_pct
            50.0,   // open_loop_speed_down_pct
            vex::deg, 100.0, 100.0, 15.0,  // units, sync, decel_threshold, decel_min
            vex::hold,                   // stop_brake_type
            true,    // stall_stop_enabled：堵转停止开启
            5.0,     // stall_velocity_rpm：两电机都低于此速度判定堵转
            200,     // stall_grace_ms：起动宽限（此时间内不检测）
            100,     // stall_confirm_ms：连续低于阈值多久判定堵转
        })),
        // 框住机构：两个气缸（电磁阀 ADI A / ADI C），B 边沿切换 框住/松开
        // 上电初始 = 展开（松开/不框住）
        dual_pneumatic(basic::mechanism::dual_pneumatic_init({
            {brain.ThreeWirePort.A},  // 气缸 A 电磁阀
            {brain.ThreeWirePort.C},  // 气缸 B 电磁阀
            true,                     // engaged_signal：高电平=框住
            basic::mechanism::DualPneumaticMode::kReleased,  // 上电初始：展开（松开）
        })) {}

  /// 初始化 IMU：标定 + 归零（与其它机器人一致；未安装时直接跳过）
  void calibrate_inertial_sensor() {
    if (!imu.installed()) {
      return;
    }
    imu.calibrate();
    while (imu.isCalibrating()) {
      vex::wait(5, vex::msec);
    }
    imu.resetHeading();
    imu.resetRotation();
  }

  void show_ready() {
    controller.Screen.setCursor(5, 1);
    controller.Screen.print("      bed ready!");
  }
};

}  // namespace basic::hardware::bed

#endif  // BASIC_SRC_HARDWARE_BED_ROBOT_HARDWARE_H_
