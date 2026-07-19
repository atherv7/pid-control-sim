#pragma once

class PIDControl {
public:
  PIDControl() = default;

  PIDControl(double k_p, double k_i, double k_d, double integral_min,
             double integral_max, double output_min, double output_max)
      : k_p_{k_p}, k_i_{k_i}, k_d_{k_d}, integral_min_{integral_min},
        integral_max_{integral_max}, output_min_{output_min},
        output_max_{output_max} {}

  auto compute(double setpoint, double measured_value, double dt) -> double;

private:
  double k_p_{};
  double k_i_{};
  double k_d_{};

  double integral_{};
  double integral_min_{};
  double integral_max_{};

  double output_min_{};
  double output_max_{};

  double prev_measurement_{};
  double prev_error_{};

  [[nodiscard]] auto clamp(double value, double min_value, double max_value)
      -> double;

  auto compute_proportional_term(double error) -> double;

  auto compute_integral_term(double error, double dt) -> double;

  auto compute_derivative_term(double measured_value, double dt) -> double;
};
