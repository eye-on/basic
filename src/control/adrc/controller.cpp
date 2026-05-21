#include "controller.hpp"

#include <algorithm>
#include <cmath>

namespace adrc {

Controller::Controller() : Controller(Config()) {}

Controller::Controller(const Config& cfg) { set_config(cfg); }

void Controller::set_config(const Config& cfg) {
  cfg_ = cfg;
  sanitize_config();

  td_.set_config(cfg_.td);
  eso_.set_config(cfg_.eso);
  nlesf_.set_config(cfg_.nlesf);
  reset(cfg_.init_reference, cfg_.init_measurement);
}

void Controller::reset(double init_reference, double init_measurement) {
  td_.reset(init_reference, 0.0);
  eso_.reset(init_measurement);
  last_u_ = 0.0;
}

Controller::Result Controller::update(double reference, double measurement) {
  Result result;
  result.reference = reference;
  result.measurement = measurement;
  result.control = last_u_;

  if (!std::isfinite(reference) || !std::isfinite(measurement)) {
    return result;
  }

  result.td = td_.update(reference);
  result.eso = eso_.update(measurement, last_u_);
  result.e1 = result.td.x1 - result.eso.z1;
  result.e2 = result.td.x2 - result.eso.z2;
  result.nlesf = nlesf_.compute(result.e1, result.e2, result.eso.z3, cfg_.b0);

  const double scaled_u = cfg_.kt * result.nlesf.u;
  result.control = clamp_value(scaled_u, cfg_.output_min, cfg_.output_max);
  last_u_ = result.control;
  return result;
}

Controller::Result Controller::update_error(double error) {
  return update(0.0, -error);
}

double Controller::clamp_value(double x, double lo, double hi) {
  return std::max(lo, std::min(x, hi));
}

void Controller::sanitize_config() {
  const double kEps = 1e-9;
  if (cfg_.output_min > cfg_.output_max) {
    std::swap(cfg_.output_min, cfg_.output_max);
  }
  if (std::fabs(cfg_.b0) < kEps) {
    cfg_.b0 = (cfg_.b0 >= 0.0) ? kEps : -kEps;
  }
  cfg_.eso.b0 = cfg_.b0;
}

}  // namespace adrc
