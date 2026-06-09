#include "controller.hpp"

#include <algorithm>
#include <cmath>

namespace basic::control::pid {

Pid::Pid() : Pid(Config()) {}

Pid::Pid(const Config& cfg) {
  config_ = cfg;
  sanitize_config();
  bind_update_function();
  reset();
}

void Pid::set_config(const Config& cfg) {
  config_ = cfg;
  sanitize_config();
  bind_update_function();
  reset();
}

void Pid::reset() {
  integral_ = 0;
  prev_error_ = 0;
  prev_error2_ = 0;
  deriv_ = 0.0;
  output_ = 0.0;
  inc_output_ = 0.0;
}

void Pid::set_type(Type type) {
  config_.type = type;
  bind_update_function();
  reset();
}

double Pid::p_linear(const Config& cfg, double error) {
  return cfg.kp * error;
}

double Pid::p_i(double ki, double integral) {
  return ki * integral;
}

double Pid::p_d(double kd, double deriv) {
  return kd * deriv;
}

Pid::Result Pid::update(double setpoint, double measurement) {
  if (!std::isfinite(setpoint) || !std::isfinite(measurement)) {
    return {};
  }

  double error = setpoint - measurement;
  if (std::fabs(error) < config_.deadzone) {
    error = 0.0;
  }

  return (this->*update_func_)(error);
}

void Pid::bind_update_function() {
  update_func_ = (config_.type == Type::kIncremental)
                     ? &Pid::update_incremental
                     : &Pid::update_positional;
}

Pid::Result Pid::update_positional(double error) {
  Result result;

  integral_ = clamp(integral_ + error, config_.i_term_min, config_.i_term_max);
  deriv_ = error - prev_error_;

  result.p = p_linear(config_, error);
  result.i = p_i(config_.ki, integral_);
  result.d = p_d(config_.kd, deriv_);
  result.ctrl = clamp(result.p + result.i + result.d, config_.out_min, config_.out_max);

  prev_error_ = error;
  output_ = result.ctrl;
  return result;
}

Pid::Result Pid::update_incremental(double error) {
  Result result;

  // △u = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
  result.p = config_.kp * (error - prev_error_);
  result.i = config_.ki * error;
  result.d = config_.kd * (error - 2.0 * prev_error_ + prev_error2_);

  const double delta_u = result.p + result.i + result.d;
  result.ctrl = clamp(inc_output_ + delta_u, config_.out_min, config_.out_max);

  // 防积分饱和：若输出被钳位，则停止累加
  if (result.ctrl == inc_output_ + delta_u) {
    inc_output_ = result.ctrl;
  }

  prev_error2_ = prev_error_;
  prev_error_ = error;
  output_ = result.ctrl;
  return result;
}

double Pid::clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(x, hi));
}

void Pid::sanitize_config() {
  if (!std::isfinite(config_.kp)) {
    config_.kp = 0.0;
  }
  if (!std::isfinite(config_.ki)) {
    config_.ki = 0.0;
  }
  if (!std::isfinite(config_.kd)) {
    config_.kd = 0.0;
  }
  if (config_.out_min > config_.out_max) {
    std::swap(config_.out_min, config_.out_max);
  }
  if (config_.i_term_min > config_.i_term_max) {
    std::swap(config_.i_term_min, config_.i_term_max);
  }
  if (config_.deadzone < 0.0 || !std::isfinite(config_.deadzone)) {
    config_.deadzone = 0.0;
  }
}

}  // namespace basic::control::pid
