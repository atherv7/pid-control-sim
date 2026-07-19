#include "pid-control-sim/pid.hpp"
#include <algorithm>
#include <stdexcept>

auto PIDControl::compute(double setpoint, double measured_value, double dt)
    -> double {
  if (dt <= 0.0) {
    throw std::invalid_argument("PIDControl::compute: dt must be positive");
  }

  double error{setpoint - measured_value};

  double P{this->compute_proportional_term(error)};

  double I{this->compute_integral_term(error, dt)};

  double D{this->compute_derivative_term(measured_value, dt)};

  double output{P + I + D};
  output = clamp(output, this->output_min_, this->output_max_);

  this->prev_error_ = error;
  this->prev_measurement_ = measured_value;

  return output;
}

[[nodiscard]] auto PIDControl::clamp(double value, double min_value,
                                     double max_value) -> double {
  return std::max(min_value, std::min(max_value, value));
}

auto PIDControl::compute_proportional_term(double error) -> double {
  return this->k_p_ * error;
}

auto PIDControl::compute_integral_term(double error, double dt) -> double {
  double new_integral{this->integral_ + error * dt};
  this->integral_ =
      this->clamp(new_integral, this->integral_min_, this->integral_max_);
  return this->k_i_ * this->integral_;
}

auto PIDControl::compute_derivative_term(double measured_value, double dt)
    -> double {
  double d_measurement{(measured_value - this->prev_measurement_) / dt};
  return this->k_d_ * d_measurement;
}
