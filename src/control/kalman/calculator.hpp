#ifndef BASIC_SRC_CONTROL_KALMAN_CALCULATOR_HPP_
#define BASIC_SRC_CONTROL_KALMAN_CALCULATOR_HPP_

#include <limits>

namespace basic::control::kalman {

class KalmanCalculator {
 public:
  KalmanCalculator();
  KalmanCalculator(double process_noise, double measurement_noise,
                   double initial_covariance = 1.0);

  void reset();
  void reset(double initial_estimate,
             double covariance = std::numeric_limits<double>::quiet_NaN());

  double update(double measurement);

  void set_process_noise(double process_noise);
  void set_measurement_noise(double measurement_noise);
  void set_initial_covariance(double initial_covariance);

  double state() const { return estimate_; }
  double covariance() const { return covariance_; }
  bool initialized() const { return is_initialized_; }

 private:
  static constexpr double kMinPositive = 1e-9;
  static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

  static double sanitize_positive(double value);

  double process_noise_ = 1e-3;
  double measurement_noise_ = 1e-2;
  double initial_covariance_ = 1.0;

  double estimate_ = kNan;
  double covariance_ = 1.0;
  bool is_initialized_ = false;
};

}  // namespace basic::control::kalman

#endif
