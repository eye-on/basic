#include "control/gearbox/software_gearbox.hpp"

#include <cmath>

namespace basic::control {

SoftwareGearbox::SoftwareGearbox() {
  apply_config(config_);
  reset();
}

SoftwareGearbox::SoftwareGearbox(const Config& config) {
  apply_config(config);
  reset();
}

void SoftwareGearbox::set_config(const Config& config) {
  const double current_input_angle = input_angle_;
  const bool was_initialized = initialized_;

  apply_config(config);
  if (was_initialized) {
    reset(current_input_angle);
  } else {
    reset();
  }
}

void SoftwareGearbox::reset() {
  initialized_ = false;
  previous_input_wrapped_ = wrap_angle(config_.initial_input_angle, config_.input_period_degrees);
  input_angle_ = 0.0;
  input_angle_wrapped_ = previous_input_wrapped_;
  input_delta_ = 0.0;
  output_angle_ = 0.0;
  output_angle_wrapped_ = 0.0;
  output_delta_ = 0.0;
}

void SoftwareGearbox::reset(double initial_input_angle) {
  initialized_ = true;
  previous_input_wrapped_ = wrap_angle(initial_input_angle, config_.input_period_degrees);
  input_angle_ = initial_input_angle;
  input_angle_wrapped_ = previous_input_wrapped_;
  input_delta_ = 0.0;
  update_output_state();
  output_delta_ = 0.0;
}

bool SoftwareGearbox::update(double input_angle_degrees) {
  const double wrapped_input = wrap_angle(input_angle_degrees, config_.input_period_degrees);

  if (!initialized_) {
    reset(wrapped_input);
    return false;
  }

  input_delta_ = unwrap_delta(wrapped_input, previous_input_wrapped_, config_.input_period_degrees);
  previous_input_wrapped_ = wrapped_input;
  input_angle_wrapped_ = wrapped_input;
  input_angle_ += input_delta_;

  const double previous_output_angle = output_angle_;
  update_output_state();
  output_delta_ = output_angle_ - previous_output_angle;
  return true;
}

double SoftwareGearbox::output_from_input(double input_angle) const {
  return input_angle / config_.ratio;
}

double SoftwareGearbox::input_from_output(double output_angle) const {
  return output_angle * config_.ratio;
}

double SoftwareGearbox::sanitize_positive(double value, double fallback) {
  if (!std::isfinite(value) || value <= kMinPositive) {
    return fallback;
  }
  return value;
}

double SoftwareGearbox::sanitize_ratio(double value) {
  if (!std::isfinite(value)) {
    return 1.0;
  }
  if (std::fabs(value) <= kMinPositive) {
    return value >= 0.0 ? 1.0 : -1.0;
  }
  return value;
}

double SoftwareGearbox::wrap_angle(double angle, double period) {
  const double wrapped = std::fmod(angle, period);
  return wrapped < 0.0 ? wrapped + period : wrapped;
}

double SoftwareGearbox::unwrap_delta(double current, double previous, double period) {
  double delta = current - previous;
  const double half_period = period * 0.5;
  if (delta > half_period) {
    delta -= period;
  } else if (delta < -half_period) {
    delta += period;
  }
  return delta;
}

void SoftwareGearbox::apply_config(const Config& config) {
  config_ = config;
  config_.input_period_degrees =
      sanitize_positive(config.input_period_degrees, kDefaultPeriodDegrees);
  config_.output_period_degrees =
      sanitize_positive(config.output_period_degrees, kDefaultPeriodDegrees);
  config_.ratio = sanitize_ratio(config.ratio);
  config_.initial_input_angle =
      wrap_angle(config.initial_input_angle, config_.input_period_degrees);
}

void SoftwareGearbox::update_output_state() {
  output_angle_ = input_angle_ / config_.ratio;
  output_angle_wrapped_ = wrap_angle(output_angle_, config_.output_period_degrees);
}

}  // namespace basic::control
