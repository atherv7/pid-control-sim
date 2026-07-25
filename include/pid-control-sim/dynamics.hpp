#pragma once

class Dynamics {
public:
  Dynamics(double moment_of_inertia, double arm_length)
      : moment_of_inertia_{moment_of_inertia}, arm_length_{arm_length} {}

  void step(double thrust_front, double thrust_back, double dt);

  void apply_impulse(double delta_angular_momentum);

  [[nodiscard]] auto pitch_angle() const -> double;

  [[nodiscard]] auto pitch_rate() const -> double;

private:
  double moment_of_inertia_;
  double arm_length_;
  double pitch_angle_{};
  double pitch_rate_{};
};
