#pragma once

#include <cstdint>
class Dynamics {
public:
  Dynamics(double moment_of_inertia, double arm_length);

  void stop(double thrust_front, double thrust_back, double dt);

  void apply_impulse(double delta_angular_momentum);

  [[nodiscard]] auto pitch_angle() const -> int64_t;

  [[nodiscard]] auto pitch_rate() const -> int64_t;

private:
  double moment_of_inertia_;
  double arm_length_;
  double pitch_angle_{};
  double pitch_rate_{};
};
