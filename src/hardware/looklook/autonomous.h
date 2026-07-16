#ifndef BASIC_SRC_HARDWARE_LOOKLOOK_AUTONOMOUS_H_
#define BASIC_SRC_HARDWARE_LOOKLOOK_AUTONOMOUS_H_

#include "chassis/heading_hold.h"
#include "hardware/looklook/robot_hardware.h"
#include "hardware/looklook/robot_state.h"
#include "vex.h"

namespace basic::hardware::looklook::autonomous {

inline constexpr double kWheelDiameterMm = 158.75;
inline constexpr double kPi = 3.141592653589793;
inline constexpr double kSqrt2 = 1.4142135623730951;
inline constexpr double kEffectiveCircumferenceMm = kPi * kWheelDiameterMm / kSqrt2;
inline constexpr double kWheelDegPerMm = 360.0 / kEffectiveCircumferenceMm;

inline constexpr double kDefaultSpeedPct = 50.0;
inline constexpr int kRefreshTimeMs = 10;

void move_forward(RobotHardware& hardware, double distance_mm, double speed_pct = kDefaultSpeedPct);
void move_backward(RobotHardware& hardware, double distance_mm, double speed_pct = kDefaultSpeedPct);
void move_left(RobotHardware& hardware, double distance_mm, double speed_pct = kDefaultSpeedPct);
void move_right(RobotHardware& hardware, double distance_mm, double speed_pct = kDefaultSpeedPct);

void run_routine(RobotHardware& hardware, RobotState& state, vex::competition& competition);

}  // namespace basic::hardware::looklook::autonomous

#endif  // BASIC_SRC_HARDWARE_LOOKLOOK_AUTONOMOUS_H_
