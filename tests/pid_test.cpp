#include "pid-control-sim/pid.hpp"
#include <cstdint>
#include <gtest/gtest.h>

namespace {
constexpr int64_t kWideMin = -1'000'000;
constexpr int64_t kWideMax = 1'000'000;
}; // namespace

TEST(PIDControl, ZeroGainsProduceZeroOutput) {
  PIDControl pid(
      /*k_p=*/0,
      /*k_i=*/0,
      /*k_d=*/0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_EQ(pid.compute(/*setpoint=*/100, /*measured_value=*/0, /*dt=*/1), 0);
  EXPECT_EQ(pid.compute(/*setpoint=*/-50, /*measured_value=*/25, /*dt=*/5), 0);
  EXPECT_EQ(pid.compute(/*setpoint=*/0, /*measured_value=*/0, /*dt=*/1), 0);
}

TEST(PIDControl, ProportionalOnlyMatchesKpTimesError) {
  PIDControl pid(
      /*k_p=*/3,
      /*k_i=*/0,
      /*k_d=*/0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_EQ(pid.compute(/*setpoint=*/100, /*measured_value=*/20, /*dt=*/1),
            240);
}

TEST(PIDControl, ProportionalOnlyHandlesNegativeError) {
  PIDControl pid(
      /*k_p=*/2,
      /*k_i=*/0,
      /*k_d=*/0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_EQ(pid.compute(/*setpoint=*/10, /*measured_value=*/50, /*dt=*/1), -80);
}

TEST(PIDControl, IntegralAccumulatesAcrossCalls) {
  PIDControl pid(
      /*k_p=*/0,
      /*k_i=*/1,
      /*k_d=*/0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_EQ(pid.compute(/*setpoint=*/10, /*measured_value=*/0, /*dt=*/1), 10);
  EXPECT_EQ(pid.compute(/*setpoint=*/10, /*measured_value=*/0, /*dt=*/1), 20);
  EXPECT_EQ(pid.compute(/*setpoint=*/10, /*measured_value=*/5, /*dt=*/2), 30);
}

TEST(PIDControl, IntegralClampsToUpperBound) {
  PIDControl pid(
      /*k_p=*/0,
      /*k_i=*/1,
      /*k_d=*/0,
      /*integral_min=*/-5,
      /*integral_max=*/15,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_EQ(pid.compute(/*setpoint=*/10, /*measured_value=*/0, /*dt=*/1), 10);
  EXPECT_EQ(pid.compute(/*setpoint=*/10, /*measured_value=*/0, /*dt=*/1), 15);
  EXPECT_EQ(pid.compute(/*setpoint=*/10, /*measured_value=*/5, /*dt=*/2), 15);
}

TEST(PIDControl, IntegralClampsToLowerBound) {
  PIDControl pid(
      /*k_p=*/0,
      /*k_i=*/1,
      /*k_d=*/0,
      /*integral_min=*/-15,
      /*integral_max=*/15,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_EQ(pid.compute(/*setpoint=*/5, /*measured_value=*/10, /*dt=*/1), -5);
  EXPECT_EQ(pid.compute(/*setpoint=*/5, /*measured_value=*/10, /*dt=*/1), -10);
  EXPECT_EQ(pid.compute(/*setpoint=*/5, /*measured_value=*/10, /*dt=*/1), -15);
  EXPECT_EQ(pid.compute(/*setpoint=*/5, /*measured_value=*/10, /*dt=*/1), -15);
}

TEST(PIDControl, DerivativeIsZerpWhenMeasurementUnchangedFromDefault) {
  PIDControl pid(
      /*k_p=*/0,
      /*k_i=*/0,
      /*k_d=*/5,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_EQ(pid.compute(/*setpoint=*/0, /*measured_value=*/0, /*dt=*/1), 0);
}

TEST(PIDControl, DerivativeReactsToMeasurementChange) {
  PIDControl pid(
      /*k_p=*/0,
      /*k_i=*/0,
      /*k_d=*/2,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_EQ(pid.compute(/*setpoint=*/0, /*measured_value=*/10, /*dt=*/1), 20);
  EXPECT_EQ(pid.compute(/*setpoint=*/0, /*measured_value=*/30, /*dt=*/2), 20);
}

TEST(PIDControl, DerivativeIntegralDivisionTruncatesTowardZero) {
  PIDControl pid(
      /*k_p=*/0,
      /*k_i=*/0,
      /*k_d=*/1,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  pid.compute(/*setpoint=*/0, /*measured_value=*/0, /*dt=*/1);

  EXPECT_EQ(pid.compute(/*setpoint=*/0, /*measured_value=*/7, /*dt=*/2), 3);
}

TEST(PIDControl, OutputClampsToUpperBound) {
  PIDControl pid(
      /*k_p=*/10,
      /*k_i=*/0,
      /*k_d=*/0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/-50,
      /*output_max=*/50);

  EXPECT_EQ(pid.compute(/*setpoint=*/100, /*measured_value=*/0, /*dt=*/1), 50);
}

TEST(PIDControl, OutputClampsToLowerBound) {
  PIDControl pid(
      /*k_p=*/10,
      /*k_i=*/0,
      /*k_d=*/0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/-50,
      /*output_max=*/50);

  EXPECT_EQ(pid.compute(/*setpoint=*/0, /*measured_value=*/100, /*dt=*/1), -50);
}

TEST(PIDControl, OutputWithinBoundsIsUnclamped) {
  PIDControl pid(
      /*k_p=*/1,
      /*k_i=*/0,
      /*k_d=*/0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/-50,
      /*output_max=*/50);

  EXPECT_EQ(pid.compute(/*setpoint=*/10, /*measured_value=*/0, /*dt=*/1), 10);
}

TEST(PIDControl, CombinedTermsAcrossMultipleSteps) {
  PIDControl pid(
      /*k_p=*/2,
      /*k_i=*/1,
      /*k_d=*/1,
      /*integral_min=*/-1'000,
      /*integral_max=*/1'000,
      /*output_min=*/-1'000,
      /*output_max=*/1'000);

  EXPECT_EQ(pid.compute(/*setpoint=*/20, /*measured_value=*/0, /*dt=*/1), 60);
  EXPECT_EQ(pid.compute(/*setpoint=*/20, /*measured_value=*/10, /*dt=*/1), 60);
  EXPECT_EQ(pid.compute(/*setpoint=*/20, /*measured_value=*/15, /*dt=*/2), 52);
}

TEST(PIDControl, DtZeroTrapsOnIntegerDivideByZero) {
  PIDControl pid(
      /*k_p=*/1,
      /*k_i=*/1,
      /*k_d=*/1,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_DEATH(
      { pid.compute(/*setpoint=*/10, /*measured_value=*/0, /*dt=*/0); }, "");
}
