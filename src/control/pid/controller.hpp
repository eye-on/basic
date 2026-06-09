#ifndef BASIC_SRC_CONTROL_PID_CONTROLLER_HPP_
#define BASIC_SRC_CONTROL_PID_CONTROLLER_HPP_

#include <limits>

namespace basic::control::pid {

enum class Type {
  kPosition,
  kIncremental,
};

class Pid {
 public:
  struct Config {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double out_min = -std::numeric_limits<double>::infinity();
    double out_max = std::numeric_limits<double>::infinity();
    double i_term_min = -std::numeric_limits<double>::infinity();
    double i_term_max = std::numeric_limits<double>::infinity();
    double deadzone = 0.0;
    Type type = Type::kPosition;
  };

  struct Result {
    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
    double ctrl = 0.0;
  };

  Pid();
  explicit Pid(const Config& cfg);

  void set_config(const Config& cfg);
  const Config& config() const { return config_; }

  void set_type(Type type);

  void reset();

  Result update(double setpoint, double measurement);

  double prev_error() const { return prev_error_; }
  double prev_error2() const { return prev_error2_; }
  double integral() const { return integral_; }
  double derivative() const { return deriv_; }
  double output() const { return output_; }

 private:
  static double clamp(double x, double lo, double hi);
  void sanitize_config();
  void bind_update_function();

  static double p_linear(const Config& cfg, double error);
  static double p_i(double ki, double integral);
  static double p_d(double kd, double deriv);

  using UpdateFunc = Result (Pid::*)(double error);
  UpdateFunc update_func_ = &Pid::update_positional;

  Result update_positional(double error);
  Result update_incremental(double error);

  Config config_;
  double integral_ = 0.0;
  double prev_error_ = 0.0;
  double prev_error2_ = 0.0;
  double deriv_ = 0.0;
  double output_ = 0.0;
  double inc_output_ = 0.0;
};

}  // namespace basic::control::pid

#endif
