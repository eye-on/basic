#ifndef BASIC_SRC_CONTROL_SOFTWARE_GEARBOX_HPP_
#define BASIC_SRC_CONTROL_SOFTWARE_GEARBOX_HPP_

#include <limits>

namespace basic::control {

class SoftwareGearbox {
 public:
  struct Config {
    double input_period_degrees = 360.0;
    double output_period_degrees = 360.0;
    double ratio = 3.0;
    double initial_input_angle = 0.0;
  };

  SoftwareGearbox();
  explicit SoftwareGearbox(const Config& config);

  void set_config(const Config& config);
  const Config& config() const { return config_; }

  void reset();
  void reset(double initial_input_angle);

  bool update(double input_angle_degrees);

  bool initialized() const { return initialized_; }
  double input_angle() const { return input_angle_; }
  double input_angle_wrapped() const { return input_angle_wrapped_; }
  double input_delta() const { return input_delta_; }
  double output_angle() const { return output_angle_; }
  double output_angle_wrapped() const { return output_angle_wrapped_; }
  double output_delta() const { return output_delta_; }

 private:
  static constexpr double kDefaultPeriodDegrees = 360.0;
  static constexpr double kMinPositive = 1e-9;
  static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

  static double sanitize_positive(double value, double fallback);
  static double sanitize_ratio(double value);
  static double wrap_angle(double angle, double period);
  static double unwrap_delta(double current, double previous, double period);

  void apply_config(const Config& config);
  void update_output_state();

  Config config_{};
  bool initialized_ = false;
  double previous_input_wrapped_ = 0.0;
  double input_angle_ = 0.0;
  double input_angle_wrapped_ = 0.0;
  double input_delta_ = 0.0;
  double output_angle_ = 0.0;
  double output_angle_wrapped_ = 0.0;
  double output_delta_ = 0.0;
};

}  // namespace basic::control

#endif
