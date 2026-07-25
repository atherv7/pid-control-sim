#include "pid-control-sim/dynamics.hpp"

void Dynamics::step(double thrust_front, double thrust_back, double dt) {
  double torque{(thrust_back - thrust_front) * arm_length_};
  double angular_acceleration{torque / moment_of_inertia_};

  pitch_rate_ += angular_acceleration * dt;
  pitch_angle_ += pitch_rate_ * dt;
}

void Dynamics::apply_impulse(double delta_angular_momentum) {
  pitch_rate_ += delta_angular_momentum / moment_of_inertia_;
}

[[nodiscard]] auto Dynamics::pitch_angle() const -> double {
  return pitch_angle_;
}

[[nodiscard]] auto Dynamics::pitch_rate() const -> double {
  return pitch_rate_;
}
