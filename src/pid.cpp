#include "pid-control-sim/pid.hpp"
#include <algorithm>
#include <cstdint>

auto PIDControl::compute(int64_t setpoint, int64_t measured_value, int64_t dt)
    -> int64_t {
  int64_t error{setpoint - measured_value};

  int64_t P{this->compute_proportional_term(error)};

  int64_t I{this->compute_integral_term(error, dt)};

  int64_t D{this->compute_derivative_term(measured_value, dt)};

  int64_t output{P + I + D};
  output = clamp(output, this->output_min_, this->output_max_);

  this->prev_error_ = error;
  this->prev_measurement_ = measured_value;

  return output;
}

[[nodiscard]] auto PIDControl::clamp(int64_t value, int64_t min_value,
                                     int64_t max_value) -> int64_t {
  return std::max(min_value, std::min(max_value, value));
}

auto PIDControl::compute_proportional_term(int64_t error) -> int64_t {
  return this->k_p_ * error;
}

auto PIDControl::compute_integral_term(int64_t error, int64_t dt) -> int64_t {
  int64_t new_integral{this->integral_ + error * dt};
  this->integral_ =
      this->clamp(new_integral, this->integral_min_, this->integral_max_);
  return this->k_i_ * this->integral_;
}

auto PIDControl::compute_derivative_term(int64_t measured_value, int64_t dt)
    -> int64_t {
  int64_t d_measurement{(measured_value - this->prev_measurement_) / dt};
  return this->k_d_ * d_measurement;
}
