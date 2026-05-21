#ifndef ADRC_NLESF_HPP_
#define ADRC_NLESF_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace adrc {

class NLESF {
 public:
  struct Config {
    double k1 = 50.0;
    double k2 = 5.0;
    double alpha1 = 0.75;
    double alpha2 = 1.25;
    double delta = 0.01;
    double u_min = -std::numeric_limits<double>::infinity();
    double u_max = std::numeric_limits<double>::infinity();
  };

  struct Output {
    double u0 = 0.0;
    double u = 0.0;
  };

  NLESF() : NLESF(Config()) {}

  explicit NLESF(const Config& cfg) { set_config(cfg); }

  void set_config(const Config& cfg) {
    cfg_ = cfg;
    sanitize_config();
  }

  const Config& config() const { return cfg_; }

  static double fal(double e, double alpha, double delta) {
    const double abs_e = std::fabs(e);
    if (abs_e <= delta) {
      return e / std::pow(delta, 1.0 - alpha);
    }
    return std::pow(abs_e, alpha) * sign(e);
  }

  Output compute(double e1, double e2, double z3, double b0) const {
    const double kEps = 1e-9;
    const double safe_b0 =
        (std::fabs(b0) < kEps) ? ((b0 >= 0.0) ? kEps : -kEps) : b0;

    Output output;
    output.u0 = cfg_.k1 * fal(e1, cfg_.alpha1, cfg_.delta) +
                cfg_.k2 * fal(e2, cfg_.alpha2, cfg_.delta);
    output.u = clamp_value((output.u0 - z3) / safe_b0, cfg_.u_min, cfg_.u_max);
    return output;
  }

 private:
  static double sign(double x) {
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
  }

  static double clamp_value(double x, double lo, double hi) {
    return std::max(lo, std::min(x, hi));
  }

  void sanitize_config() {
    const double kEps = 1e-9;
    cfg_.delta = std::max(cfg_.delta, kEps);
    cfg_.alpha1 = std::max(cfg_.alpha1, kEps);
    cfg_.alpha2 = std::max(cfg_.alpha2, kEps);
    if (cfg_.u_min > cfg_.u_max) {
      std::swap(cfg_.u_min, cfg_.u_max);
    }
  }

  Config cfg_;
};

}  // namespace adrc

#endif
