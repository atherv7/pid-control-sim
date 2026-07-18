#pragma once

#include <cstdint>

class PIDControl {
public:
  auto compute(int64_t setpoint, int64_t measured_value, int64_t dt) -> int64_t;

private:
  int64_t k_p_;
  int64_t k_i_;
  int64_t k_d_;

  int64_t integral_;
  int64_t integral_min_;
  int64_t integral_max_;

  int64_t output_min_;
  int64_t output_max_;

  int64_t prev_measurement_;
  int64_t prev_error_;

  [[nodiscard]] auto clamp(int64_t value, int64_t min_value, int64_t max_value)
      -> int64_t;

  auto compute_proportional_term(int64_t error) -> int64_t;

  auto compute_integral_term(int64_t error, int64_t dt) -> int64_t;

  auto compute_derivative_term(int64_t measured_value, int64_t dt) -> int64_t;
};
