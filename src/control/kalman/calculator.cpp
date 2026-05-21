#include "control/kalman/calculator.hpp"

#include <algorithm>
#include <cmath>

namespace basic::control::kalman {

KalmanCalculator::KalmanCalculator() { reset(); }

KalmanCalculator::KalmanCalculator(double process_noise, double measurement_noise,
                                   double initial_covariance)
    : process_noise_(sanitize_positive(process_noise)),
      measurement_noise_(sanitize_positive(measurement_noise)),
      initial_covariance_(sanitize_positive(initial_covariance)) {
  reset();
}

void KalmanCalculator::reset() {
  estimate_ = kNan;
  covariance_ = initial_covariance_;
  is_initialized_ = false;
}

void KalmanCalculator::reset(double initial_estimate, double covariance) {
  estimate_ = initial_estimate;
  if (std::isfinite(covariance) && covariance > kMinPositive) {
    covariance_ = covariance;
  } else {
    covariance_ = initial_covariance_;
  }
  is_initialized_ = std::isfinite(initial_estimate);
}

double KalmanCalculator::update(double measurement) {
  if (!std::isfinite(measurement)) {
    return kNan;
  }

  if (!is_initialized_) {
    estimate_ = measurement;
    covariance_ = initial_covariance_;
    is_initialized_ = true;
    return estimate_;
  }

  covariance_ = std::max(covariance_ + process_noise_, kMinPositive);
  const double kalman_gain = covariance_ / (covariance_ + measurement_noise_);
  estimate_ += kalman_gain * (measurement - estimate_);
  covariance_ = std::max((1.0 - kalman_gain) * covariance_, kMinPositive);
  return estimate_;
}

void KalmanCalculator::set_process_noise(double process_noise) {
  process_noise_ = sanitize_positive(process_noise);
}

void KalmanCalculator::set_measurement_noise(double measurement_noise) {
  measurement_noise_ = sanitize_positive(measurement_noise);
}

void KalmanCalculator::set_initial_covariance(double initial_covariance) {
  initial_covariance_ = sanitize_positive(initial_covariance);
  if (!is_initialized_) {
    covariance_ = initial_covariance_;
  }
}

double KalmanCalculator::sanitize_positive(double value) {
  return std::max(value, kMinPositive);
}

}  // namespace basic::control::kalman
