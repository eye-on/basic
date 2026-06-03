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

KalmanCalculator::KalmanCalculator(double A, double B, double process_noise,
                                   double measurement_noise, double K,
                                   double initial_covariance)
    : A_(A),
      B_(B),
      K_(std::max(K, 0.0)),
      process_noise_(sanitize_positive(process_noise)),
      measurement_noise_(sanitize_positive(measurement_noise)),
      initial_covariance_(sanitize_positive(initial_covariance)) {
  reset();
}

void KalmanCalculator::reset() {
  estimate_ = kNan;
  covariance_ = initial_covariance_;
  is_initialized_ = false;
  predicted_ = false;
}

void KalmanCalculator::reset(double initial_estimate, double covariance) {
  estimate_ = initial_estimate;
  if (std::isfinite(covariance) && covariance > kMinPositive) {
    covariance_ = covariance;
  } else {
    covariance_ = initial_covariance_;
  }
  is_initialized_ = std::isfinite(initial_estimate);
  predicted_ = false;
}

void KalmanCalculator::predict(double input) {
  if (!is_initialized_) return;
  estimate_ = A_ * estimate_ + B_ * input;
  covariance_ = std::max(A_ * A_ * covariance_ + process_noise_, kMinPositive);
  predicted_ = true;
}

double KalmanCalculator::update(double measurement) {
  if (!std::isfinite(measurement)) {
    return kNan;
  }

  if (!is_initialized_) {
    estimate_ = measurement;
    covariance_ = initial_covariance_;
    is_initialized_ = true;
    predicted_ = false;
    return estimate_;
  }

  if (!predicted_) {
    covariance_ = std::max(covariance_ + process_noise_, kMinPositive);
  }
  predicted_ = false;

  const double kalman_gain = (K_ > kMinPositive)
      ? K_
      : covariance_ / (covariance_ + measurement_noise_);
  estimate_ += kalman_gain * (measurement - estimate_);
  covariance_ = std::max((1.0 - kalman_gain) * covariance_, kMinPositive);
  return estimate_;
}

void KalmanCalculator::set_A(double A) { A_ = A; }
void KalmanCalculator::set_B(double B) { B_ = B; }
void KalmanCalculator::set_K(double K) { K_ = std::max(K, 0.0); }

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
