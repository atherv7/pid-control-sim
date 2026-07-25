#include "pid-control-sim/dynamics.hpp"
#include <gtest/gtest.h>
#include <sys/types.h>

namespace {
constexpr double kDefaultMomentOfInertia{2.0};
constexpr double kDefaultArmLength{0.5};
} // namespace

TEST(Dynamics, InitialStateIsZeror) {
  Dynamics dyn(kDefaultMomentOfInertia, kDefaultArmLength);
  EXPECT_DOUBLE_EQ(dyn.pitch_angle(), 0.0);
  EXPECT_DOUBLE_EQ(dyn.pitch_rate(), 0.0);
}

TEST(Dynamics, EqualThrustProducesNoMotion) {
  Dynamics dyn(kDefaultMomentOfInertia, kDefaultArmLength);

  for (int i{}; i < 5; ++i) {
    dyn.step(/*thrust_front=*/3.0, /*thrust_back=*/3.0, /*dt=*/1.0);
    EXPECT_DOUBLE_EQ(dyn.pitch_angle(), 0.0);
    EXPECT_DOUBLE_EQ(dyn.pitch_rate(), 0.0);
  }
}

TEST(Dynamics, ZeroThrustBothMotorsProducesNoMotion) {
  Dynamics dyn(kDefaultMomentOfInertia, kDefaultArmLength);
  dyn.step(/*thrust_front=*/0.0, /*thrust_back=*/0.0, /*dt=*/2.0);
  EXPECT_DOUBLE_EQ(dyn.pitch_rate(), 0.0);
  EXPECT_DOUBLE_EQ(dyn.pitch_angle(), 0.0);
}

TEST(Dynamics, ConstantTorqueMatchesDiscreteClosedForm) {
  Dynamics dyn(kDefaultMomentOfInertia, kDefaultArmLength);
  constexpr double kAccel{1.0};
  constexpr double kDt{1.0};

  for (int i{1}; i <= 3; ++i) {
    dyn.step(/*thrust_front=*/0.0, /*thrust_back=*/4.0, kDt);
    EXPECT_DOUBLE_EQ(dyn.pitch_rate(), i * kAccel * kDt);
    EXPECT_DOUBLE_EQ(dyn.pitch_angle(), kAccel * kDt * kDt * i * (i + 1) / 2);
  }
}

TEST(Dynamics, ConstantTorqueWithNonUnitDt) {
  Dynamics dyn(kDefaultMomentOfInertia, kDefaultArmLength);
  constexpr double kAccel{1.0};
  constexpr double kDt{0.5};

  for (int i{1}; i <= 2; ++i) {
    dyn.step(/*thrust_front=*/1.0, /*thrust_back=*/5.0, kDt);
    EXPECT_DOUBLE_EQ(dyn.pitch_rate(), i * kAccel * kDt);
    EXPECT_DOUBLE_EQ(dyn.pitch_angle(), kAccel * kDt * kDt * i * (i + 1) / 2);
  }
}

TEST(Dynamics, ReversingThrustDifferenceFlipsSign) {
  Dynamics positive(kDefaultMomentOfInertia, kDefaultArmLength);
  Dynamics negative(kDefaultMomentOfInertia, kDefaultArmLength);

  positive.step(/*thrust_front=*/0.0, /*thrust_back=*/4.0, /*dt=*/1.0);
  negative.step(/*thrust_front=*/4.0, /*thrust_back=*/0.0, /*dt=*/1.0);

  EXPECT_DOUBLE_EQ(positive.pitch_rate(), -negative.pitch_rate());
  EXPECT_DOUBLE_EQ(positive.pitch_angle(), -negative.pitch_angle());
}

TEST(Dynamics, ApplyImpulseChangesRateNotAngleImmediately) {
  Dynamics dyn(kDefaultMomentOfInertia, kDefaultArmLength);
  EXPECT_DOUBLE_EQ(dyn.pitch_rate(), 0.0);

  dyn.apply_impulse(/*delta_angular_momentum=*/4.0);

  EXPECT_DOUBLE_EQ(dyn.pitch_rate(), 2.0);
  EXPECT_DOUBLE_EQ(dyn.pitch_angle(), 0.0);
}

TEST(Dynamics, ImpulseAffectsAngleOnNextStep) {
  Dynamics dyn(kDefaultMomentOfInertia, kDefaultArmLength);
  dyn.apply_impulse(/*delta_angular_momentum=*/4.0);

  dyn.step(/*thrust_front=*/0.0, /*thrust_back=*/0.0, /*dt=*/0.5);

  EXPECT_DOUBLE_EQ(dyn.pitch_rate(), 2.0);
  EXPECT_DOUBLE_EQ(dyn.pitch_angle(), 1.0);
}

TEST(Dynamics, MultipleImpulsesAccumulate) {
  Dynamics dyn(kDefaultMomentOfInertia, kDefaultArmLength);
  dyn.apply_impulse(/*delta_angular_momentum=*/2.0);
  dyn.apply_impulse(/*delta_angular_momentum=*/-4.0);
  dyn.apply_impulse(/*delta_angular_momentum=*/6.0);

  EXPECT_DOUBLE_EQ(dyn.pitch_rate(), 2.0);
  EXPECT_DOUBLE_EQ(dyn.pitch_angle(), 0.0);
}

TEST(Dynamics, NegativeImpulseDecreasesRate) {
  Dynamics dyn(kDefaultMomentOfInertia, kDefaultArmLength);
  dyn.apply_impulse(/*delta_angular_momentum=*/-6.0);
  EXPECT_DOUBLE_EQ(dyn.pitch_rate(), -3.0);
}

TEST(Dynamics, ImpulseDuringOngoingTorqueCombinesCorrectly) {
  Dynamics dyn(kDefaultMomentOfInertia, kDefaultArmLength);

  dyn.step(/*thrust_front=*/0.0, /*thrust_back=*/4.0, /*dt=*/1.0);
  EXPECT_DOUBLE_EQ(dyn.pitch_rate(), 1.0);
  EXPECT_DOUBLE_EQ(dyn.pitch_angle(), 1.0);

  dyn.apply_impulse(/*delta_angular_momentum=*/2.0);
  EXPECT_DOUBLE_EQ(dyn.pitch_rate(), 2.0);
  EXPECT_DOUBLE_EQ(dyn.pitch_angle(), 1.0);

  dyn.step(/*thrust_front=*/0.0, /*thrust_back=*/4.0, /*dt=*/1.0);
  EXPECT_DOUBLE_EQ(dyn.pitch_rate(), 3.0);
  EXPECT_DOUBLE_EQ(dyn.pitch_angle(), 4.0);
}
