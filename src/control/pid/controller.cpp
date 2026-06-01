#include "controller.hpp"

#include <algorithm>
#include <cmath>

namespace basic::control::pid {

Pid::Pid() : Pid(Config()) {}

Pid::Pid(const Config& cfg) {
  cfg_ = cfg;
  sanitize_config();
  update_calculator();
  reset();
}

void Pid::set_config(const Config& cfg) {
  cfg_ = cfg;
  sanitize_config();
  update_calculator();
  reset();
}

void Pid::reset() {
  i_ = 0;
  err_ = 0;
  d_ = 0.0;
  ctrl_ = 0.0;
}

void Pid::set_mode(Mode mode) {
  cfg_.mode = mode;
  update_calculator();
}

void Pid::set_calculator(Calculator calc) {
  if (calc != nullptr) {
    calculator_ = calc;
  }
}

void Pid::update_calculator() {
  switch (cfg_.mode) {
    case Mode::kLogarithmic:
      calculator_ = p_logarithmic;
      break;
    case Mode::kLinear:
    default:
      calculator_ = p_linear;
      break;
  }
}

double Pid::p_linear(double kp, double /* log_gain */, double err) {
  return kp * err;
}

double Pid::p_logarithmic(double kp, double log_gain, double err) {
  if (std::fabs(err) < 1e-9) {
    return 0.0;
  }
  const double sign = err >= 0.0 ? 1.0 : -1.0;
  const double abs_err = std::fabs(err);
  return kp * sign * (log_gain * std::log1p(abs_err));
}

double Pid::p_i(double ki, double integral) {
  return ki * integral;
}

double Pid::p_d(double kd, double deriv) {
  return kd * deriv;
}

Pid::Result Pid::update(double expection, double measurement) {
  Result result;
  result.ctrl = ctrl_;
  
  if (!std::isfinite(expection) || !std::isfinite(measurement)) {
    return result;
  }

  double err = expection - measurement;
  if (std::fabs(err) < cfg_.deadzone) {
    err = 0.0;
  }

  i_ = clamp(i_ + err, cfg_.i_term_min, cfg_.i_term_max);
  
  d_ = err - err_;

  result.p = calculator_(cfg_.kp, cfg_.log_gain, err);
  result.i = p_i(cfg_.ki, i_);
  result.d = p_d(cfg_.kd, d_);
  result.ctrl = clamp(result.p + result.i + result.d, cfg_.out_min, cfg_.out_max);

  err_ = err;
  ctrl_ = result.ctrl;
  return result;
}

double Pid::clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(x, hi));
}

void Pid::sanitize_config() {
  if (!std::isfinite(cfg_.kp)) {
    cfg_.kp = 0.0;
  }
  if (!std::isfinite(cfg_.ki)) {
    cfg_.ki = 0.0;
  }
  if (!std::isfinite(cfg_.kd)) {
    cfg_.kd = 0.0;
  }
  if (!std::isfinite(cfg_.log_gain) || cfg_.log_gain <= 0.0) {
    cfg_.log_gain = 1.0;
  }

  if (cfg_.out_min > cfg_.out_max) {
    std::swap(cfg_.out_min, cfg_.out_max);
  }
  if (cfg_.i_term_min > cfg_.i_term_max) {
    std::swap(cfg_.i_term_min, cfg_.i_term_max);
  }
  if (cfg_.deadzone < 0.0 || !std::isfinite(cfg_.deadzone)) {
    cfg_.deadzone = 0.0;
  }

}

}  // namespace basic::control::pid
