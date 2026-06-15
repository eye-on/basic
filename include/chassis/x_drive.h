#ifndef BASIC_INCLUDE_X_DRIVE_H_
#define BASIC_INCLUDE_X_DRIVE_H_

#include "chassis/arcade_drive.h"
#include "control/pid/controller.hpp"

namespace basic::chassis {

/// ?????????????????????????
/// x ???? forward ????????y ???? strafe ??????
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

/// ????????????y?? PID ??????????
template <std::size_t Count, std::size_t... Indices>
std::array<basic::control::pid::Pid, Count> make_pid_array_impl(
    const std::array<basic::control::pid::Pid::Config, Count>& configs,
    std::index_sequence<Indices...>) {
  return {{basic::control::pid::Pid{configs[Indices]}...}};
}

template <std::size_t Count>
std::array<basic::control::pid::Pid, Count> make_pid_array(
    const std::array<basic::control::pid::Pid::Config, Count>& configs) {
  return make_pid_array_impl(configs, std::make_index_sequence<Count>{});
}

/// ??? PID ???????????????????
/// ??????????? PID ???????????????????
template <std::size_t Count>
void apply_group_output_pid(
    std::array<vex::motor, Count>& motors,
    std::array<basic::control::pid::Pid, Count>& pids,
    double pct,
    vex::brakeType brake_type) {
  for (std::size_t i = 0; i < Count; ++i) {
    if (pct != 0.0) {
      const double current_vel = motors[i].velocity(vex::pct);
      //printf("%.2f,%.2f\n",pct,current_vel);
      if (std::abs(pct) < 2 && std::abs(current_vel) < 2) {
        pids[i].reset();
      }
      const auto result = pids[i].update(pct, current_vel);
      basic::control::velocitycontrol(motors[i], result.ctrl, vex::pct);
      // ?????????????????????????????????? PID????????????
      }
      else {
      basic::control::stopcontrol(motors[i], brake_type);
      pids[i].reset();}
  } 
}

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

/// ??????????????X-Drive?????????
/// ?????????????????????????
struct XDriveCommand {
  int forward_input_pct{0};
  int strafe_input_pct{0};
  int turn_input_pct{0};
  vex::brakeType stop_brake_type{vex::coast};
};

/// ??????????????X-Drive????????
struct XDriveState {
  double fl_pct{0.0};
  double fr_pct{0.0};
  double bl_pct{0.0};
  double br_pct{0.0};
  vex::brakeType stop_brake_type{vex::coast};
  XDriveOdometry odometry{};
};

/// ??????????????X-Drive??????
/// ?????????????????????????????????X ???????
template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
struct XDriveConfig {
  std::array<basic::device::MotorConfig, FlCount> fl_motors;  // ??????????
  std::array<basic::device::MotorConfig, FrCount> fr_motors;  // ??????????
  std::array<basic::device::MotorConfig, BlCount> bl_motors;  // ???????????
  std::array<basic::device::MotorConfig, BrCount> br_motors;  // ??????????
  int deadzone{10};
  /// ??????? PID ????????????????? PID ????????
  std::array<basic::control::pid::Pid::Config, FlCount> fl_pid_configs{};
  std::array<basic::control::pid::Pid::Config, FrCount> fr_pid_configs{};
  std::array<basic::control::pid::Pid::Config, BlCount> bl_pid_configs{};
  std::array<basic::control::pid::Pid::Config, BrCount> br_pid_configs{};
};

/// ??????????????X-Drive??
/// ???????????????????? X ????????? mecanum ?????????????
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
        fl_pid_(detail::make_pid_array(config.fl_pid_configs)),
        fr_pid_(detail::make_pid_array(config.fr_pid_configs)),
        bl_pid_(detail::make_pid_array(config.bl_pid_configs)),
        br_pid_(detail::make_pid_array(config.br_pid_configs)) {}

  /// ??????Front-Left????????????
  std::array<vex::motor, FlCount>& fl_motors() {
    return fl_motors_;
  }

  const std::array<vex::motor, FlCount>& fl_motors() const {
    return fl_motors_;
  }

  /// ??????Front-Right????????????
  std::array<vex::motor, FrCount>& fr_motors() {
    return fr_motors_;
  }

  const std::array<vex::motor, FrCount>& fr_motors() const {
    return fr_motors_;
  }

  /// ???????Back-Left????????????
  std::array<vex::motor, BlCount>& bl_motors() {
    return bl_motors_;
  }

  const std::array<vex::motor, BlCount>& bl_motors() const {
    return bl_motors_;
  }

  /// ??????Back-Right????????????
  std::array<vex::motor, BrCount>& br_motors() {
    return br_motors_;
  }

  const std::array<vex::motor, BrCount>& br_motors() const {
    return br_motors_;
  }

  int deadzone() const {
    return deadzone_;
  }

  /// ????? PID ????????????
  std::array<basic::control::pid::Pid, FlCount>& fl_pid() {
    return fl_pid_;
  }

  const std::array<basic::control::pid::Pid, FlCount>& fl_pid() const {
    return fl_pid_;
  }

  /// ????? PID ????????????
  std::array<basic::control::pid::Pid, FrCount>& fr_pid() {
    return fr_pid_;
  }

  const std::array<basic::control::pid::Pid, FrCount>& fr_pid() const {
    return fr_pid_;
  }

  /// ?????? PID ????????????
  std::array<basic::control::pid::Pid, BlCount>& bl_pid() {
    return bl_pid_;
  }

  const std::array<basic::control::pid::Pid, BlCount>& bl_pid() const {
    return bl_pid_;
  }

  /// ????? PID ????????????
  std::array<basic::control::pid::Pid, BrCount>& br_pid() {
    return br_pid_;
  }

  const std::array<basic::control::pid::Pid, BrCount>& br_pid() const {
    return br_pid_;
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
  std::array<basic::control::pid::Pid, FlCount> fl_pid_;
  std::array<basic::control::pid::Pid, FrCount> fr_pid_;
  std::array<basic::control::pid::Pid, BlCount> bl_pid_;
  std::array<basic::control::pid::Pid, BrCount> br_pid_;
  int deadzone_{10};
  XDriveState state_;
};

/// ??????????????????
template <std::size_t FlCount, std::size_t FrCount,
          std::size_t BlCount, std::size_t BrCount>
XDrive<FlCount, FrCount, BlCount, BrCount> x_drive_init(
    const XDriveConfig<FlCount, FrCount, BlCount, BrCount>& config) {
  return XDrive<FlCount, FrCount, BlCount, BrCount>(config);
}

/// ??????????????????????????????????
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
      stop_brake_type,
  };
}

/// ???????????????????????????
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
  constexpr double kRpmToMps = 3.14159265358979 * 0.1062 / 60.0;
  const double fl_actual_rpm =
      detail::average_group_velocity(chassis.fl_motors(), vex::rpm);
  const double fr_actual_rpm =
      detail::average_group_velocity(chassis.fr_motors(), vex::rpm);
  const double bl_actual_rpm =
      detail::average_group_velocity(chassis.bl_motors(), vex::rpm);
  const double br_actual_rpm =
      detail::average_group_velocity(chassis.br_motors(), vex::rpm);
  const double fl_actual_pct =
      detail::average_group_velocity(chassis.fl_motors(), vex::velocityUnits::pct);
  const double fr_actual_pct =
      detail::average_group_velocity(chassis.fr_motors(), vex::velocityUnits::pct);
  const double bl_actual_pct =
      detail::average_group_velocity(chassis.bl_motors(), vex::velocityUnits::pct);
  const double br_actual_pct =
      detail::average_group_velocity(chassis.br_motors(), vex::velocityUnits::pct);
  const double fl_mps = fl_actual_rpm * kRpmToMps;
  const double fr_mps = fr_actual_rpm * kRpmToMps;
  const double bl_mps = bl_actual_rpm * kRpmToMps;
  const double br_mps = br_actual_rpm * kRpmToMps;

  detail::update_odometry(
      chassis.state().odometry,
      fl_mps,
      fr_mps,
      bl_mps,
      br_mps,
      vex::timer::system());

  printf(
      "%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
      static_cast<int>(vex::timer::system()),
      fl_pct, fl_actual_pct,
      fr_pct, fr_actual_pct,
      bl_pct, bl_actual_pct,
      br_pct, br_actual_pct);
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

  const double forward = apply_deadzone(command.forward_input_pct);
  const double strafe = apply_deadzone(command.strafe_input_pct);
  const double turn = apply_deadzone(command.turn_input_pct);

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

/// ????????????????????????????
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