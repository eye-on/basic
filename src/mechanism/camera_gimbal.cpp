#include "mechanism/camera_gimbal.h"

#include "control/motor_control.h"

#include <algorithm>

namespace basic::mechanism {

namespace {

using basic::control::stopcontrol;
using basic::control::velocitycontrol;

vex::motor make_motor(const basic::device::MotorConfig& config) {
  return vex::motor{config.port, config.gear_ratio, config.reversed};
}

double clamp_pct(double value) {
  return std::max(-100.0, std::min(value, 100.0));
}

void sync_state(CameraGimbal& mechanism) {
  mechanism.state().motor_position_deg = mechanism.motor().position(vex::deg);
  mechanism.state().motor_installed = mechanism.motor().installed();
  mechanism.state().motor_spinning = mechanism.motor().isSpinning();
}

void apply_output(CameraGimbal& mechanism) {
  if (mechanism.state().motor_pct != 0.0) {
    velocitycontrol(mechanism.motor(), mechanism.state().motor_pct, vex::pct);
    sync_state(mechanism);
    return;
  }

  stopcontrol(mechanism.motor(), vex::hold);
  sync_state(mechanism);
}

}  // namespace

CameraGimbal::CameraGimbal(const CameraGimbalConfig& config)
    : config_(config), motor_(make_motor(config.motor)) {
  sync_state(*this);
}

const CameraGimbalConfig& CameraGimbal::config() const { return config_; }

vex::motor& CameraGimbal::motor() { return motor_; }

const vex::motor& CameraGimbal::motor() const { return motor_; }

CameraGimbalState& CameraGimbal::state() { return state_; }

const CameraGimbalState& CameraGimbal::state() const { return state_; }

CameraGimbal camera_gimbal_init(const CameraGimbalConfig& config) {
  return CameraGimbal(config);
}

void camera_gimbal_update(CameraGimbal& mechanism, const CameraGimbalCommand& command) {
  camera_gimbal_set_output(mechanism, command.motor_pct);
}

void camera_gimbal_set_output(CameraGimbal& mechanism, double motor_pct) {
  mechanism.state().motor_pct = clamp_pct(motor_pct);
  apply_output(mechanism);
}

void camera_gimbal_refresh_state(CameraGimbal& mechanism) {
  sync_state(mechanism);
}

void camera_gimbal_stop(CameraGimbal& mechanism, vex::brakeType brake_type) {
  mechanism.state().motor_pct = 0.0;
  stopcontrol(mechanism.motor(), brake_type);
  sync_state(mechanism);
}

CameraGimbalState& camera_gimbal_state(CameraGimbal& mechanism) {
  return mechanism.state();
}

const CameraGimbalState& camera_gimbal_state(const CameraGimbal& mechanism) {
  return mechanism.state();
}

}  // namespace basic::mechanism
