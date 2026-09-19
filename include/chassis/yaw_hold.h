#ifndef BASIC_INCLUDE_YAW_HOLD_H_
#define BASIC_INCLUDE_YAW_HOLD_H_

#include "control/pid/controller.hpp"
#include "vex.h"

#include <cmath>

namespace basic::chassis {

/// IMU 航向保持（摇杆积分目标 + 航向闭环，**一直实时运行**）
///
/// 思路：
///   1) 把旋转摇杆的输入做时域积分，得到"目标 yaw"——**唯一能改变目标的只有这个摇杆**；
///      摇杆在死区内时目标冻结，于是被推动/摩擦造成的偏航都会被闭环拉回来；
///   2) 用 IMU 实测 yaw 与目标 yaw 之差做 PID，输出修正量叠加到 turn 通道；
///   3) 闭环不受"是否在驱动"影响：静止、平移、原地旋转时都在修正，
///      所以原地被推歪也会自己转回目标（重锚定见 R1 键）。
///
/// 单位约定：
///   - yaw 用 IMU 的 rotation(deg)（0-360 连续、无跳变），不用 heading()；
///   - 修正量单位 = 底盘 turn 通道的 pct（叠加在曲线整形之后，线性生效）。
struct YawHoldConfig {
  basic::control::pid::Pid::Config pid;      // 误差 deg → 修正 pct
  int turn_deadzone{0};                       // 已整形 turn 指令的死区（pct，通常 0）
  double deg_per_sec_per_turn_pct{5.6};       // 标定值：1 pct turn 指令 ≈ 多少 °/s
  double max_correction_pct{20.0};            // 修正限幅（pct）
  double max_turn_lead_deg{15.0};             // 打杆转向时允许的"目标超前量"（防积分跑飞）
  double rate_damping_pct_per_dps{0.0};       // 用 IMU 角速度做阻尼（pct/(deg/s)，0=关）
  bool reanchor_on_release{true};             // 松杆瞬间接受当前车头为目标（消除松手后又转几度）
  int sign{1};                                // 方向修正（±1；实测反了就改 -1）
  bool enabled{true};                         // 运行时可开关（R2）
};

/// 本周期输入
struct YawHoldInput {
  // 底盘**曲线整形之后**的 turn 指令（pct）：驾驶员真正要的转向量。
  // 用它与底盘共用同一个量，才不会出现"环按原始摇杆积分、底盘按整形值转"的错配。
  double turn_cmd_pct{0.0};
  double yaw_deg{0.0};      // IMU rotation(deg)
  double yaw_rate_dps{0.0}; // IMU gyroRate(zaxis, dps)（仅阻尼用）
  bool imu_ready{false};    // IMU 已安装且标定完成
};

struct YawHoldState {
  double target_yaw_deg{0.0};
  double yaw_deg{0.0};
  double error_deg{0.0};
  double correction_pct{0.0};
  bool active{false};
  bool anchored{false};
  bool was_turning{false};
};

struct YawHold {
  YawHoldConfig config;
  YawHoldState state;
  basic::control::pid::Pid pid;
  int last_ms{0};
};

inline YawHold yaw_hold_init(const YawHoldConfig& config = {}) {
  YawHold hold;
  hold.config = config;
  hold.pid = basic::control::pid::Pid(config.pid);
  return hold;
}

/// 把目标重新锚定到当前航向（例如上电、或松开按钮后重新对齐）
inline void yaw_hold_reset(YawHold& hold, double yaw_deg) {
  hold.state.target_yaw_deg = yaw_deg;
  hold.state.yaw_deg = yaw_deg;
  hold.state.error_deg = 0.0;
  hold.state.correction_pct = 0.0;
  hold.state.active = false;
  hold.state.anchored = true;
  hold.pid.reset();
}

/// 计算本周期应叠加到 turn 通道的修正量（pct）
inline double yaw_hold_update(YawHold& hold, const YawHoldInput& input) {
  YawHoldState& st = hold.state;
  const YawHoldConfig& cfg = hold.config;

  const int now = static_cast<int>(vex::timer::system());
  int dt_ms = (hold.last_ms == 0) ? 10 : (now - hold.last_ms);
  if (dt_ms < 1) {
    dt_ms = 1;
  }
  if (dt_ms > 50) {
    dt_ms = 50;  // 长时间未调用（阻塞/暂停）按一个周期处理
  }
  hold.last_ms = now;

  st.yaw_deg = input.yaw_deg;

  if (!cfg.enabled || !input.imu_ready) {
    hold.pid.reset();
    st.active = false;
    st.correction_pct = 0.0;
    return 0.0;
  }

  if (!st.anchored) {
    yaw_hold_reset(hold, input.yaw_deg);
  }

  // ① 目标 yaw 的速率基准 = 底盘整形之后的 turn 指令（驾驶员真正要的转向量）
  //    这样"环要转多少"和"底盘正在转多少"是同一个量，不会互相较劲
  const double turn_cmd = input.turn_cmd_pct;
  const bool turning = std::abs(turn_cmd) > static_cast<double>(cfg.turn_deadzone);
  if (turning) {
    st.target_yaw_deg += turn_cmd * cfg.deg_per_sec_per_turn_pct *
                         static_cast<double>(dt_ms) * 0.001;
  } else if (st.was_turning && cfg.reanchor_on_release) {
    // 松杆瞬间：把当前车头接受为目标（消除"松手后自己再转几度"的不跟手感）
    yaw_hold_reset(hold, input.yaw_deg);
    st.was_turning = false;
    return 0.0;
  }
  st.was_turning = turning;

  // ② 打杆期间限制"目标超前量"：闭环跟不上时不至于把目标越积越远
  //    （松杆时目标已冻结，这里不再动它——被推歪只能靠闭环转回来）
  double error = st.target_yaw_deg - input.yaw_deg;
  if (turning && std::abs(error) > cfg.max_turn_lead_deg) {
    st.target_yaw_deg = input.yaw_deg +
                        (error > 0.0 ? cfg.max_turn_lead_deg
                                     : -cfg.max_turn_lead_deg);
    error = st.target_yaw_deg - input.yaw_deg;
  }
  st.error_deg = error;

  // ③ 航向闭环：**每周期都执行**（静止/平移/原地旋转都锁航向）
  //    误差 deg → 修正 pct（可用 IMU 角速度做阻尼）
  double correction = hold.pid.update(st.target_yaw_deg, input.yaw_deg).ctrl;
  correction -= cfg.rate_damping_pct_per_dps * input.yaw_rate_dps;
  correction *= static_cast<double>(cfg.sign);
  if (correction > cfg.max_correction_pct) {
    correction = cfg.max_correction_pct;
  }
  if (correction < -cfg.max_correction_pct) {
    correction = -cfg.max_correction_pct;
  }

  st.correction_pct = correction;
  st.active = true;
  return correction;
}

}  // namespace basic::chassis

#endif  // BASIC_INCLUDE_YAW_HOLD_H_
