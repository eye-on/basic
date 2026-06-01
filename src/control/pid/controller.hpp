#ifndef BASIC_SRC_CONTROL_PID_CONTROLLER_HPP_
#define BASIC_SRC_CONTROL_PID_CONTROLLER_HPP_

#include <limits>

namespace basic::control::pid {

enum class Mode {
  kLinear,
  kLogarithmic,
};

using Calculator = double (*)(double kp, double log_gain, double err);

class Pid {
 public:
  struct Config {
    Mode mode = Mode::kLinear;
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double log_gain = 1.0;
    double out_min = -std::numeric_limits<double>::infinity();
    double out_max = std::numeric_limits<double>::infinity();
    double i_term_min = -std::numeric_limits<double>::infinity();
    double i_term_max = std::numeric_limits<double>::infinity();
    double deadzone = 0.0;
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
  const Config& config() const { return cfg_; }

  void set_mode(Mode mode);
  void set_calculator(Calculator calc);

  void reset();

  Result update(double expection, double measurement);

  double last_err() const { return err_; }
  double last_i() const { return i_; }
  double last_d() const { return d_; }
  double last_ctrl() const { return ctrl_; }

 private:
  static double clamp(double x, double lo, double hi);
  void sanitize_config();
  void update_calculator();

  static double p_linear(double kp, double log_gain, double err);
  static double p_logarithmic(double kp, double log_gain, double err);
  static double p_i(double ki, double integral);
  static double p_d(double kd, double deriv);

  Config cfg_;
  Calculator calculator_ = p_linear;
  double i_ = 0.0;
  double err_ = 0.0;
  double d_ = 0.0;
  double ctrl_ = 0.0;
};

}  // namespace basic::control::pid

#endif
