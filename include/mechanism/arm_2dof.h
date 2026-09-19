#ifndef BASIC_INCLUDE_MECHANISM_ARM_2DOF_H_
#define BASIC_INCLUDE_MECHANISM_ARM_2DOF_H_

#include "device_config.h"
#include "hardware/shared/state_types.h"

namespace basic::mechanism {

/// 二自由度机械臂控制模式
enum class Arm2DofMode {
  kOpenLoopVelocity,   // 开环速度：按键按住 = 以配置速度持续转动
  kClosedLoopPosition, // 闭环位置：按键边沿在两个预设位置间切换（home ↔ alt），
                       // 走电机固件位置环（spinToPosition）
  kStallLimit,         // 堵转找限位：两个位置都有机械硬限位，
                       // 朝目标方向定速转动 → 速度≈0 判定到位（不依赖编码器/齿比）
};

struct Arm2DofConfig {
  basic::device::MotorConfig motor1_a;    // 关节 1 电机组 A
  basic::device::MotorConfig motor1_b;    // 关节 1 电机组 B（双电机同驱关节 1）
  basic::device::MotorConfig motor2;      // 关节 2 电机
  Arm2DofMode mode{Arm2DofMode::kOpenLoopVelocity};  // 位置闭环/开环速度
  double velocity_speed_pct{40.0};         // 开环速度幅值（按住按键时）
  vex::rotationUnits position_units{vex::deg};  // 闭环位置单位
  double position_speed_pct{30.0};         // 闭环运动速度
  vex::brakeType stop_brake_type{vex::hold};  // 停止模式（构造时 setStopping 持久生效）
  // 闭环两个预设位置（上电初始位置为 0，即 home）
  double joint1_home{0.0};   // 关节 1 初始位置
  double joint1_alt{220.0};  // 关节 1 另一位置（展开位）
  double joint2_home{0.0};   // 关节 2 初始位置
  double joint2_alt{200.0};  // 关节 2 另一位置（展开位）
  // 联锁：关节 1 未展开时禁止关节 2 展开；
  //       关节 2 未收回时禁止关节 1 收回（防机械碰撞）
  bool joint12_interlock{true};

  // 堵转找限位模式（kStallLimit）参数
  int joint1_alt_sign{1};            // 关节 1 朝 alt 限位转动的 pct 符号（+1/-1）
  int joint2_alt_sign{1};            // 关节 2 朝 alt 限位转动的 pct 符号（+1/-1）
  double stall_speed_pct{40.0};      // 顶限位时的驱动速度
  double stall_velocity_rpm{5.0};    // 判定堵转的速度阈值（两个电机都低于此值）
  int stall_grace_ms{200};           // 启动宽限：此时间内不检测（避免起动瞬间误判）
  int stall_confirm_ms{150};         // 连续低于阈值多久判定到位
  int stall_timeout_ms{4000};        // 单次行程安全超时（超时停机并标记未到达）
};

struct Arm2DofCommand {
  // 开环指令：±1 = 方向，0 = 停（由 command_from_controller 从按键电平生成）
  int joint1_direction{0};  // up=+1 / down=-1（开环模式用）
  int joint2_direction{0};  // x=+1 / b=-1（开环模式用）
  // 闭环位置指令：按下沿 → 切到对应预设位置（位置/堵转模式用）
  bool joint1_to_alt{false};   // up 边沿  → 去 joint1_alt
  bool joint1_to_home{false};  // down 边沿 → 回 joint1_home
  bool joint2_to_alt{false};   // x 边沿   → 去 joint2_alt
  bool joint2_to_home{false};  // b 边沿   → 回 joint2_home
  // 单键整臂切换：按下沿 → 收缩 ↔ 展开（两关节按序动作）
  bool arm_toggle{false};      // y 边沿
};

struct Arm2DofState {
  Arm2DofMode mode{Arm2DofMode::kOpenLoopVelocity};
  double joint1_position{0.0};  // 关节 1 位置（电机组双电机取平均，position_units）
  double joint2_position{0.0};  // 关节 2 电机当前位置
  double joint1_target{0.0};    // 闭环目标（position_units）
  double joint2_target{0.0};
  bool joint1_at_alt{false};    // 期望状态：true = 要去 alt（展开），false = home（收回）
  bool joint2_at_alt{false};
  bool joint2_deploy_blocked{false};   // 上一次关节2展开请求被联锁拒绝
  bool joint1_retract_blocked{false};  // 上一次关节1收回请求被联锁拒绝
  // 堵转找限位模式的运行状态
  bool joint1_alt_reached{false};  // 已知物理状态：true = 已顶在 alt 限位
  bool joint2_alt_reached{false};
  bool joint1_moving{false};       // 正在朝目标限位运动
  bool joint2_moving{false};
  bool joint1_stalled{false};      // 上一次运动以"堵转到位"结束
  bool joint2_stalled{false};
  bool joint1_timeout{false};      // 行程超时已放弃（需新的位置指令才重试）
  bool joint2_timeout{false};
  int joint1_stall_since_ms{0};    // 速度低于阈值的起始时刻（0 = 未进入）
  int joint2_stall_since_ms{0};
  int joint1_move_start_ms{0};     // 本次行程起始时刻（起动宽限/超时用）
  int joint2_move_start_ms{0};
  // 单键整臂序列
  bool arm_extended{false};        // 整臂当前状态（true = 已展开）
  int arm_sequence_step{0};        // 0=空闲 1=正在展开 2=正在收回
  bool extend_allowed{true};       // 展开许可（外部联锁：如框住机构处于展开态时置 false）
  bool extend_blocked{false};      // 上一次展开请求被许可条件拒绝
  int joint1_direction{0};      // 最近开环指令方向
  int joint2_direction{0};
};

class Arm2Dof {
 public:
  explicit Arm2Dof(const Arm2DofConfig& config);

  vex::motor& motor1_a();
  vex::motor& motor1_b();
  vex::motor& motor2();
  const vex::motor& motor1_a() const;
  const vex::motor& motor1_b() const;
  const vex::motor& motor2() const;

  Arm2DofConfig& config();
  const Arm2DofConfig& config() const;

  Arm2DofState& state();
  const Arm2DofState& state() const;

 private:
  Arm2DofConfig config_;
  vex::motor motor1_a_;
  vex::motor motor1_b_;
  vex::motor motor2_;
  Arm2DofState state_;
};

/// 初始化二自由度机械臂
Arm2Dof arm_2dof_init(const Arm2DofConfig& config);

/// 从遥控器输入生成指令：**Y = 整臂 收缩↔展开（单键边沿，自动按序）**
/// （不映射单关节按键；单关节/开环指令由 autonomous 自行构造 Command）
Arm2DofCommand arm_2dof_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input);

/// 主更新：按模式驱动（开环 = 电平方向速度；闭环 = 固件位置环；堵转 = 开环+找限位）
void arm_2dof_update(Arm2Dof& mechanism, const Arm2DofCommand& command);

/// 切换控制模式（闭环模式需先设目标角）
void arm_2dof_set_mode(Arm2Dof& mechanism, Arm2DofMode mode);

/// 整臂展开/收缩（autonomous 用；堵转模式按序动作，完成后 state().arm_extended 更新）
void arm_2dof_set_extended(Arm2Dof& mechanism, bool extended);

/// 设置展开许可（外部联锁用；false 时拒绝展开，收回始终允许）
void arm_2dof_set_extend_allowed(Arm2Dof& mechanism, bool allowed);

/// 设置闭环目标角（position_units；autonomous 用）
void arm_2dof_set_joint1_target(Arm2Dof& mechanism, double target);
void arm_2dof_set_joint2_target(Arm2Dof& mechanism, double target);

/// 停止机械臂（默认 hold：手动模式松键后保持姿态）
void arm_2dof_stop(Arm2Dof& mechanism, vex::brakeType brake_type = vex::hold);

/// 状态访问（mutable / const）
Arm2DofState& arm_2dof_state(Arm2Dof& mechanism);
const Arm2DofState& arm_2dof_state(const Arm2Dof& mechanism);

}  // namespace basic::mechanism

#endif  // BASIC_INCLUDE_MECHANISM_ARM_2DOF_H_
