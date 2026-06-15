#ifndef BASIC_INCLUDE_MECHANISM_CAMERA_GIMBAL_H_
#define BASIC_INCLUDE_MECHANISM_CAMERA_GIMBAL_H_

#include "device_config.h"
#include "vex.h"

namespace basic::mechanism {

struct CameraGimbalConfig {
  basic::device::MotorConfig motor;
};

struct CameraGimbalCommand {
  double motor_pct{0.0};
};

struct CameraGimbalState {
  double motor_pct{0.0};
  double motor_position_deg{0.0};
  bool motor_installed{false};
  bool motor_spinning{false};
};

class CameraGimbal {
 public:
  explicit CameraGimbal(const CameraGimbalConfig& config);

  const CameraGimbalConfig& config() const;

  vex::motor& motor();
  const vex::motor& motor() const;

  CameraGimbalState& state();
  const CameraGimbalState& state() const;

 private:
  CameraGimbalConfig config_;
  vex::motor motor_;
  CameraGimbalState state_;
};

CameraGimbal camera_gimbal_init(const CameraGimbalConfig& config);

void camera_gimbal_update(CameraGimbal& mechanism, const CameraGimbalCommand& command);
void camera_gimbal_set_output(CameraGimbal& mechanism, double motor_pct);
void camera_gimbal_refresh_state(CameraGimbal& mechanism);
void camera_gimbal_stop(CameraGimbal& mechanism, vex::brakeType brake_type = vex::hold);

CameraGimbalState& camera_gimbal_state(CameraGimbal& mechanism);
const CameraGimbalState& camera_gimbal_state(const CameraGimbal& mechanism);

}  // namespace basic::mechanism

#endif  // BASIC_INCLUDE_MECHANISM_CAMERA_GIMBAL_H_
