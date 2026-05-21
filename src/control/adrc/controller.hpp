#ifndef ADRC_CONTROLLER_HPP_
#define ADRC_CONTROLLER_HPP_

#include <limits>

#include "ESO.hpp"
#include "NLESF.hpp"
#include "TD.hpp"

namespace adrc {

class Controller {
 public:
  struct Config {
    TD::Config td;
    ESO::Config eso;
    NLESF::Config nlesf;
    double b0 = 1.0;
    double kt = 1.0;
    double output_min = -std::numeric_limits<double>::infinity();
    double output_max = std::numeric_limits<double>::infinity();
    double init_reference = 0.0;
    double init_measurement = 0.0;
  };

  struct Result {
    double reference = 0.0;
    double measurement = 0.0;
    double e1 = 0.0;
    double e2 = 0.0;
    double control = 0.0;
    TD::Output td;
    ESO::Output eso;
    NLESF::Output nlesf;
  };

  Controller();
  explicit Controller(const Config& cfg);

  void set_config(const Config& cfg);
  const Config& config() const { return cfg_; }

  void reset(double init_reference = 0.0, double init_measurement = 0.0);

  Result update(double reference, double measurement);
  Result update_error(double error);

  double last_control() const { return last_u_; }
  const TD& tracking_differentiator() const { return td_; }
  const ESO& observer() const { return eso_; }
  const NLESF& feedback() const { return nlesf_; }

 private:
  static double clamp_value(double x, double lo, double hi);
  void sanitize_config();

  Config cfg_;
  TD td_;
  ESO eso_;
  NLESF nlesf_;
  double last_u_ = 0.0;
};

}  // namespace adrc

#endif
