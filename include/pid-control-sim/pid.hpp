#pragma once

#include <cstdint>

class PIDControl {
public:
  PIDControl() = default;

  PIDControl(int64_t k_p, int64_t k_i, int64_t k_d, int64_t integral_min,
             int64_t integral_max, int64_t output_min, int64_t output_max)
      : k_p_{k_p}, k_i_{k_i}, k_d_{k_d}, integral_min_{integral_min},
        integral_max_{integral_max}, output_min_{output_min},
        output_max_{output_max} {}

  auto compute(int64_t setpoint, int64_t measured_value, int64_t dt) -> int64_t;

private:
  int64_t k_p_{};
  int64_t k_i_{};
  int64_t k_d_{};

  int64_t integral_{};
  int64_t integral_min_{};
  int64_t integral_max_{};

  int64_t output_min_{};
  int64_t output_max_{};

  int64_t prev_measurement_{};
  int64_t prev_error_{};

  [[nodiscard]] auto clamp(int64_t value, int64_t min_value, int64_t max_value)
      -> int64_t;

  auto compute_proportional_term(int64_t error) -> int64_t;

  auto compute_integral_term(int64_t error, int64_t dt) -> int64_t;

  auto compute_derivative_term(int64_t measured_value, int64_t dt) -> int64_t;
};
