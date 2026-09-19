#ifndef BASIC_INCLUDE_X_DRIVE_H_
#define BASIC_INCLUDE_X_DRIVE_H_

#include "chassis/arcade_drive.h"

namespace basic::chassis {

/// 十字型全向轮底盘里程计：由四轮速度积分得到位姿
/// x 轴对应 forward 前后方向，y 轴对应 strafe 平移方向
struct XDriveOdometry {
  double x_m{0.0};
  double y_m{0.0};
  double forward_mps{0.0};
  double strafe_mps{0.0};
  double fl_mps{0.0};
  double fr_mps{0.0};
  double bl_mps{0.0};
  double br_mps{0.0};
  double fl_projected_m{0.0};
  double fr_projected_m{0.0};
  double bl_projected_m{0.0};
  double br_projected_m{0.0};
  int last_update_ms{0};
  bool initialized{false};
};

namespace detail {

template <std::size_t Count>
double average_group_velocity(
    std::array<vex::motor, Count>& motors,
    vex::velocityUnits units) {
  double sum = 0.0;
  for (vex::motor& motor : motors) {
    sum += motor.velocity(units);
  }
  return sum / static_cast<double>(Count);
}

inline void set_odometry_pose(XDriveOdometry& odometry, double x_m, double y_m) {
  odometry.x_m = x_m;
  odometry.y_m = y_m;
  odometry.fl_projected_m = 0.5 * (x_m + y_m);
  odometry.fr_projected_m = 0.5 * (x_m - y_m);
  odometry.bl_projected_m = 0.5 * (x_m - y_m);
  odometry.br_projected_m = 0.5 * (x_m + y_m);
  odometry.forward_mps = 0.0;
  odometry.strafe_mps = 0.0;
  odometry.fl_mps = 0.0;
  odometry.fr_mps = 0.0;
  odometry.bl_mps = 0.0;
  odometry.br_mps = 0.0;
  odometry.last_update_ms = vex::timer::system();
  odometry.initialized = true;
}

inline void update_odometry(
    XDriveOdometry& odometry,
    double fl_mps,
    double fr_mps,
    double bl_mps,
    double br_mps,
    int now_ms) {
  constexpr double kSqrt2 = 1.4142135623730951;
  constexpr double kSqrt2Over2 = 0.7071067811865475;

  odometry.fl_mps = fl_mps;
  odometry.fr_mps = fr_mps;
  odometry.bl_mps = bl_mps;
  odometry.br_mps = br_mps;
  odometry.forward_mps = (fl_mps + fr_mps + bl_mps + br_mps) / (2.0 * kSqrt2);
  odometry.strafe_mps = (fl_mps - fr_mps - bl_mps + br_mps) / (2.0 * kSqrt2);

  if (!odometry.initialized) {
    odometry.last_update_ms = now_ms;
    odometry.initialized = true;
    return;
  }

  int delta_ms = now_ms - odometry.last_update_ms;
  if (delta_ms < 0) {
    delta_ms = 0;
  }
  odometry.last_update_ms = now_ms;

  const double dt_s = static_cast<double>(delta_ms) * 0.001;
  if (dt_s <= 0.0) {
    return;
  }

  odometry.fl_projected_m += fl_mps * kSqrt2Over2 * dt_s;
  odometry.fr_projected_m += fr_mps * kSqrt2Over2 * dt_s;
  odometry.bl_projected_m += bl_mps * kSqrt2Over2 * dt_s;
  odometry.br_projected_m += br_mps * kSqrt2Over2 * dt_s;

  odometry.x_m =
      (odometry.fl_projected_m + odometry.fr_projected_m +
       odometry.bl_projected_m + odometry.br_projected_m) * 0.5;
  odometry.y_m =
      (odometry.fl_projected_m - odometry.fr_projected_m -
       odometry.bl_projected_m + odometry.br_projected_m) * 0.5;
}

}  // namespace detail

/// 十字型全向轮底盘（X-Drive）控制指令
struct XDriveCommand {
  int forward_input_pct{0};
  int strafe_input_pct{0};
  int turn_input_pct{0};
  /// 航向修正量（pct）：叠加到 turn 通道，**在曲线整形之后**以差分形式生效
  /// （+c 给 fl/bl，−c 给 fr/br），因此修正量与轮速线性对应——IMU 航向保持用
  double turn_correction_pct{0.0};
  vex::brakeType stop_brake_type{vex::coast};
  // 测试用：直接给定四轮输出（跳过摇杆死区/灵敏度/混合/曲线，命令值完全确定）
  // 用于固定速度复现测量（空载/半加载/落地的数据可对比）
  bool use_direct_output{false};
  double direct_fl_pct{0.0};
  double direct_fr_pct{0.0};
  double direct_bl_pct{0.0};
  double direct_br_pct{0.0};
};

/// 十字型全向轮底盘（X-Drive）状态
struct XDriveState {
  double fl_pct{0.0};
  double fr_pct{0.0};
  double bl_pct{0.0};
  double br_pct{0.0};
  double fl_rpm{0.0};  // 实测轮速（组内平均，仅记录供调试/遥测）
  double fr_rpm{0.0};
  double bl_rpm{0.0};
  double br_rpm{0.0};
  // 曲线整形之后的 turn 通道指令（pct）：驾驶员真正要的转向量
  // 供航向保持当"目标偏航角速度"的基准，保证外环和底盘用同一个量
  double turn_pct{0.0};
  vex::brakeType stop_brake_type{vex::coast};
  XDriveOdometry odometry{};
};

/// 十字型全向轮底盘（X-Drive）配置
/// 四个角各一组电机，几何上按 X 型排布
template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
struct XDriveConfig {
  std::array<basic::device::MotorConfig, FlCount> fl_motors;  // 左前电机组
  std::array<basic::device::MotorConfig, FrCount> fr_motors;  // 右前电机组
  std::array<basic::device::MotorConfig, BlCount> bl_motors;  // 左后电机组
  std::array<basic::device::MotorConfig, BrCount> br_motors;  // 右后电机组
  int deadzone{10};
  double forward_sensitivity{1.0};  // 前后灵敏度
  double strafe_sensitivity{1.0};   // 左右灵敏度
  double turn_sensitivity{1.0};     // 旋转灵敏度
  // 分轴死区覆盖（0 = 使用 deadzone 默认值；单位同摇杆 pct）
  int deadzone_forward{0};  // 前后轴（bed 映射：轴 2）
  int deadzone_strafe{0};   // 平移轴（bed 映射：轴 1）
  int deadzone_turn{0};     // 旋转轴（bed 映射：轴 4）
  // 输出方式：默认力控（pct → 电压直驱，其他机器人沿用）；
  // true = pct 直接作为电机固件速度环的目标速度（固件高频闭环，抗负载突变）
  bool use_firmware_velocity{false};
};

/// 十字型全向轮底盘（X-Drive）
/// 控制链：摇杆 → 零位偏置 → 分轴死区（平滑起算）→ 灵敏度 → X 型混合 → >100 归一化
///         → smoothstep 曲线 → 下发（固件速度环 pct / 力控电压；无软件速度环、无斜率限制）
template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
class XDrive {
 public:
  explicit XDrive(
      const XDriveConfig<FlCount, FrCount, BlCount, BrCount>& config)
      : fl_motors_(detail::make_motor_array(config.fl_motors)),
        fr_motors_(detail::make_motor_array(config.fr_motors)),
        bl_motors_(detail::make_motor_array(config.bl_motors)),
        br_motors_(detail::make_motor_array(config.br_motors)),
        deadzone_(config.deadzone),
        forward_sensitivity_(config.forward_sensitivity),
        strafe_sensitivity_(config.strafe_sensitivity),
        turn_sensitivity_(config.turn_sensitivity),
        deadzone_forward_(config.deadzone_forward),
        deadzone_strafe_(config.deadzone_strafe),
        deadzone_turn_(config.deadzone_turn),
        use_firmware_velocity_(config.use_firmware_velocity) {}

  /// 是否使用电机固件速度环（pct 作为目标速度下发）
  bool use_firmware_velocity() const { return use_firmware_velocity_; }

  /// 摇杆零位偏置（静差补偿）：上电标定后写入，输入先减去偏置再进死区
  void set_axis_offsets(int forward_offset, int strafe_offset, int turn_offset) {
    axis_offset_forward_ = forward_offset;
    axis_offset_strafe_ = strafe_offset;
    axis_offset_turn_ = turn_offset;
  }

  int axis_offset_forward() const { return axis_offset_forward_; }
  int axis_offset_strafe() const { return axis_offset_strafe_; }
  int axis_offset_turn() const { return axis_offset_turn_; }

  /// 左前（Front-Left）电机组
  std::array<vex::motor, FlCount>& fl_motors() {
    return fl_motors_;
  }

  const std::array<vex::motor, FlCount>& fl_motors() const {
    return fl_motors_;
  }

  /// 右前（Front-Right）电机组
  std::array<vex::motor, FrCount>& fr_motors() {
    return fr_motors_;
  }

  const std::array<vex::motor, FrCount>& fr_motors() const {
    return fr_motors_;
  }

  /// 左后（Back-Left）电机组
  std::array<vex::motor, BlCount>& bl_motors() {
    return bl_motors_;
  }

  const std::array<vex::motor, BlCount>& bl_motors() const {
    return bl_motors_;
  }

  /// 右后（Back-Right）电机组
  std::array<vex::motor, BrCount>& br_motors() {
    return br_motors_;
  }

  const std::array<vex::motor, BrCount>& br_motors() const {
    return br_motors_;
  }

  int deadzone() const {
    return deadzone_;
  }

  /// 分轴有效死区（覆盖值 ≤0 时回落到 deadzone）
  int deadzone_forward() const {
    return deadzone_forward_ > 0 ? deadzone_forward_ : deadzone_;
  }

  int deadzone_strafe() const {
    return deadzone_strafe_ > 0 ? deadzone_strafe_ : deadzone_;
  }

  int deadzone_turn() const {
    return deadzone_turn_ > 0 ? deadzone_turn_ : deadzone_;
  }

  double forward_sensitivity() const {
    return forward_sensitivity_;
  }

  double strafe_sensitivity() const {
    return strafe_sensitivity_;
  }

  double turn_sensitivity() const {
    return turn_sensitivity_;
  }

  XDriveState& state() {
    return state_;
  }

  const XDriveState& state() const {
    return state_;
  }

 private:
  std::array<vex::motor, FlCount> fl_motors_;
  std::array<vex::motor, FrCount> fr_motors_;
  std::array<vex::motor, BlCount> bl_motors_;
  std::array<vex::motor, BrCount> br_motors_;
  int deadzone_{10};
  double forward_sensitivity_{1.0};
  double strafe_sensitivity_{1.0};
  double turn_sensitivity_{1.0};
  int deadzone_forward_{0};  // 分轴死区覆盖（0 = 用 deadzone_）
  int deadzone_strafe_{0};
  int deadzone_turn_{0};
  // 摇杆零位偏置（静差补偿；上电标定后写入）
  int axis_offset_forward_{0};
  int axis_offset_strafe_{0};
  int axis_offset_turn_{0};
  bool use_firmware_velocity_{false};  // true = pct 作为固件速度环目标下发
  XDriveState state_;
};

/// 初始化十字型全向轮底盘
template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
XDrive<FlCount, FrCount, BlCount, BrCount> x_drive_init(
    const XDriveConfig<FlCount, FrCount, BlCount, BrCount>& config) {
  return XDrive<FlCount, FrCount, BlCount, BrCount>(config);
}

/// 从遥控器输入生成底盘控制指令（指定前后 / 平移 / 旋转轴）
inline XDriveCommand x_drive_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input,
    ControllerAxis forward_axis,
    ControllerAxis strafe_axis,
    ControllerAxis turn_axis,
    vex::brakeType stop_brake_type = vex::coast) {
  return {
      controller_axis_value(input, forward_axis),
      controller_axis_value(input, strafe_axis),
      controller_axis_value(input, turn_axis),
      0.0,
      stop_brake_type,
  };
}

/// 直接设置四轮输出
/// - 力控（默认）：pct → 电压下发；≈0 时按刹车类型停止
/// - 固件速度环：pct → 电机固件速度环目标速度；≈0 时按刹车类型停止
template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
void x_drive_set_output(
    XDrive<FlCount, FrCount, BlCount, BrCount>& chassis,
    double fl_pct,
    double fr_pct,
    double bl_pct,
    double br_pct,
    vex::brakeType brake_type) {
  chassis.state().fl_pct = fl_pct;
  chassis.state().fr_pct = fr_pct;
  chassis.state().bl_pct = bl_pct;
  chassis.state().br_pct = br_pct;
  chassis.state().stop_brake_type = brake_type;
  if (chassis.use_firmware_velocity()) {
    detail::apply_group_velocity_output(chassis.fl_motors(), fl_pct, brake_type);
    detail::apply_group_velocity_output(chassis.fr_motors(), fr_pct, brake_type);
    detail::apply_group_velocity_output(chassis.bl_motors(), bl_pct, brake_type);
    detail::apply_group_velocity_output(chassis.br_motors(), br_pct, brake_type);
    return;
  }
  detail::apply_group_output(chassis.fl_motors(), fl_pct, brake_type);
  detail::apply_group_output(chassis.fr_motors(), fr_pct, brake_type);
  detail::apply_group_output(chassis.bl_motors(), bl_pct, brake_type);
  detail::apply_group_output(chassis.br_motors(), br_pct, brake_type);
}

template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
void x_drive_update(
    XDrive<FlCount, FrCount, BlCount, BrCount>& chassis,
    const XDriveCommand& command) {
  // 测试模式：直接给定四轮输出（不做摇杆处理），实测轮速仍写入状态
  //   航向修正同样叠加（以 turn 差分形式），这样固定速度测试也能锁航向
  if (command.use_direct_output) {
    chassis.state().fl_rpm = detail::group_rpm(chassis.fl_motors());
    chassis.state().fr_rpm = detail::group_rpm(chassis.fr_motors());
    chassis.state().bl_rpm = detail::group_rpm(chassis.bl_motors());
    chassis.state().br_rpm = detail::group_rpm(chassis.br_motors());
    const double fix = command.turn_correction_pct;
    x_drive_set_output(chassis,
                       detail::clamp_pct(command.direct_fl_pct + fix),
                       detail::clamp_pct(command.direct_fr_pct - fix),
                       detail::clamp_pct(command.direct_bl_pct + fix),
                       detail::clamp_pct(command.direct_br_pct - fix),
                       command.stop_brake_type);
    return;
  }

  // 分轴死区（平滑起算）：|v| ≤ dz → 0；超过后从 0 起算并重新标定到满量程
  //   例（dz=10）：11 → 1.1%，50 → 44.4%，100 → 100%（无跳变）
  auto apply_deadzone = [](int input, int dz) -> double {
    const double v = static_cast<double>(input);
    const double mag = std::abs(v);
    if (mag <= static_cast<double>(dz)) {
      return 0.0;
    }
    const double span = 100.0 - static_cast<double>(dz);
    if (span <= 0.0) {
      return 0.0;
    }
    const double scaled = (mag - static_cast<double>(dz)) * 100.0 / span;
    return (v > 0.0) ? scaled : -scaled;
  };

  // 先减去摇杆零位偏置（静差补偿），再进分轴死区
  const double forward =
      apply_deadzone(command.forward_input_pct - chassis.axis_offset_forward(),
                     chassis.deadzone_forward()) *
      chassis.forward_sensitivity();
  const double strafe =
      apply_deadzone(command.strafe_input_pct - chassis.axis_offset_strafe(),
                     chassis.deadzone_strafe()) *
      chassis.strafe_sensitivity();
  const double turn =
      apply_deadzone(command.turn_input_pct - chassis.axis_offset_turn(),
                     chassis.deadzone_turn()) *
      chassis.turn_sensitivity();

  // fl = forward + strafe + turn
  // fr = forward - strafe - turn
  // bl = forward - strafe + turn
  // br = forward + strafe - turn
  double fl_pct = forward + strafe + turn;
  double fr_pct = forward - strafe - turn;
  double bl_pct = forward - strafe + turn;
  double br_pct = forward + strafe - turn;

  const double max_pct = std::max(
      {std::fabs(fl_pct), std::fabs(fr_pct),
       std::fabs(bl_pct), std::fabs(br_pct)});
  if (max_pct > 100.0) {
    const double scale = 100.0 / max_pct;
    fl_pct *= scale;
    fr_pct *= scale;
    bl_pct *= scale;
    br_pct *= scale;
  }

  // 曲线整形后的目标（速度指令）
  double out_fl = detail::shape_input(fl_pct);
  double out_fr = detail::shape_input(fr_pct);
  double out_bl = detail::shape_input(bl_pct);
  double out_br = detail::shape_input(br_pct);

  // 发布 turn 通道的整形后指令（纯旋转时 fl=+t → out_fl 就是它的整形值）
  // 航向保持用它当目标角速度基准：驾驶员要转多少，目标就按同样比例走
  chassis.state().turn_pct = detail::shape_input(turn);

  // 航向修正（IMU/外部闭环注入）：以 turn 差分形式叠加在曲线整形之后
  //   放在整形之前会被 smoothstep 在小信号区压扁（斜率趋 0），修正量近乎无效
  const double correction = command.turn_correction_pct;
  if (correction != 0.0) {
    out_fl += correction;
    out_bl += correction;
    out_fr -= correction;
    out_br -= correction;
    out_fl = detail::clamp_pct(out_fl);
    out_fr = detail::clamp_pct(out_fr);
    out_bl = detail::clamp_pct(out_bl);
    out_br = detail::clamp_pct(out_br);
  }

  // 实测轮速（仅记录到状态，供调试/遥测；不参与输出计算）
  chassis.state().fl_rpm = detail::group_rpm(chassis.fl_motors());
  chassis.state().fr_rpm = detail::group_rpm(chassis.fr_motors());
  chassis.state().bl_rpm = detail::group_rpm(chassis.bl_motors());
  chassis.state().br_rpm = detail::group_rpm(chassis.br_motors());

  x_drive_set_output(
      chassis,
      out_fl,
      out_fr,
      out_bl,
      out_br,
      command.stop_brake_type);
}

template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
void x_drive_stop(
    XDrive<FlCount, FrCount, BlCount, BrCount>& chassis,
    vex::brakeType brake_type = vex::coast) {
  x_drive_set_output(chassis, 0.0, 0.0, 0.0, 0.0, brake_type);
}

template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
XDriveState& x_drive_state(
    XDrive<FlCount, FrCount, BlCount, BrCount>& chassis) {
  return chassis.state();
}

template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
const XDriveState& x_drive_state(
    const XDrive<FlCount, FrCount, BlCount, BrCount>& chassis) {
  return chassis.state();
}

/// 里程计访问（可修改 / 只读）
template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
XDriveOdometry& x_drive_odometry(
    XDrive<FlCount, FrCount, BlCount, BrCount>& chassis) {
  return chassis.state().odometry;
}

template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
const XDriveOdometry& x_drive_odometry(
    const XDrive<FlCount, FrCount, BlCount, BrCount>& chassis) {
  return chassis.state().odometry;
}

template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
void x_drive_reset_odometry(
    XDrive<FlCount, FrCount, BlCount, BrCount>& chassis,
    double x_m = 0.0,
    double y_m = 0.0) {
  detail::set_odometry_pose(chassis.state().odometry, x_m, y_m);
}

}  // namespace basic::chassis

#endif  // BASIC_INCLUDE_X_DRIVE_H_
