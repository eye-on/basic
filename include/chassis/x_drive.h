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
  double turn_correction_pct{0.0};
  vex::brakeType stop_brake_type{vex::coast};
};

/// 十字型全向轮底盘（X-Drive）状态
struct XDriveState {
  double fl_pct{0.0};
  double fr_pct{0.0};
  double bl_pct{0.0};
  double br_pct{0.0};
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
};

/// 十字型全向轮底盘（X-Drive）
/// 简单速控：输出百分比直接下发为电机速度指令，不经 PID
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
        turn_sensitivity_(config.turn_sensitivity) {}

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

/// 直接设置四轮输出（简单速控：非零下发速度百分比，零则按刹车类型停止）
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
  const int deadzone = chassis.deadzone();
  auto apply_deadzone = [deadzone](int input) -> double {
    return std::abs(input) > deadzone ? static_cast<double>(input) : 0.0;
  };

  const double forward = apply_deadzone(command.forward_input_pct) * chassis.forward_sensitivity();
  const double strafe = apply_deadzone(command.strafe_input_pct) * chassis.strafe_sensitivity();
  const double turn = apply_deadzone(command.turn_input_pct) * chassis.turn_sensitivity()
                      + command.turn_correction_pct;

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

  x_drive_set_output(
      chassis,
      detail::shape_input(fl_pct),
      detail::shape_input(fr_pct),
      detail::shape_input(bl_pct),
      detail::shape_input(br_pct),
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
