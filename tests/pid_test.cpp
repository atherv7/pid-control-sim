#include "pid-control-sim/pid.hpp"
#include <gtest/gtest.h>
#include <stdexcept>

namespace {
constexpr double kWideMin = -1'000'000.0;
constexpr double kWideMax = 1'000'000.0;
}; // namespace

TEST(PIDControl, ZeroGainsProduceZeroOutput) {
  PIDControl pid(
      /*k_p=*/0.0,
      /*k_i=*/0.0,
      /*k_d=*/0.0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/100.0, /*measured_value=*/0.0, /*dt=*/1.0), 0.0);
  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/-50.0, /*measured_value=*/25.0, /*dt=*/5.0),
      0.0);
  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/0.0, /*measured_value=*/0.0, /*dt=*/1.0), 0.0);
}

TEST(PIDControl, ProportionalOnlyMatchesKpTimesError) {
  PIDControl pid(
      /*k_p=*/3.0,
      /*k_i=*/0.0,
      /*k_d=*/0.0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/100.0, /*measured_value=*/20.0, /*dt=*/1.0),
      240.0);
}

TEST(PIDControl, ProportionalOnlyHandlesNegativeError) {
  PIDControl pid(
      /*k_p=*/2.0,
      /*k_i=*/0.0,
      /*k_d=*/0.0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/10.0, /*measured_value=*/50.0, /*dt=*/1.0),
      -80.0);
}

TEST(PIDControl, IntegralAccumulatesAcrossCalls) {
  PIDControl pid(
      /*k_p=*/0.0,
      /*k_i=*/1.0,
      /*k_d=*/0.0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/10.0, /*measured_value=*/0.0, /*dt=*/1.0), 10.0);
  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/10.0, /*measured_value=*/0.0, /*dt=*/1.0), 20.0);
  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/10.0, /*measured_value=*/5.0, /*dt=*/2.0), 30.0);
}

TEST(PIDControl, IntegralClampsToUpperBound) {
  PIDControl pid(
      /*k_p=*/0.0,
      /*k_i=*/1.0,
      /*k_d=*/0.0,
      /*integral_min=*/-5.0,
      /*integral_max=*/15.0,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/10.0, /*measured_value=*/0.0, /*dt=*/1.0), 10.0);
  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/10.0, /*measured_value=*/0.0, /*dt=*/1.0), 15.0);
  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/10.0, /*measured_value=*/5.0, /*dt=*/2.0), 15.0);
}

TEST(PIDControl, IntegralClampsToLowerBound) {
  PIDControl pid(
      /*k_p=*/0.0,
      /*k_i=*/1.0,
      /*k_d=*/0.0,
      /*integral_min=*/-15.0,
      /*integral_max=*/15.0,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/5.0, /*measured_value=*/10.0, /*dt=*/1.0), -5.0);
  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/5.0, /*measured_value=*/10.0, /*dt=*/1.0),
      -10.0);
  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/5.0, /*measured_value=*/10.0, /*dt=*/1.0),
      -15.0);
  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/5.0, /*measured_value=*/10.0, /*dt=*/1.0),
      -15.0);
}

TEST(PIDControl, DerivativeIsZerpWhenMeasurementUnchangedFromDefault) {
  PIDControl pid(
      /*k_p=*/0.0,
      /*k_i=*/0.0,
      /*k_d=*/5.0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/0.0, /*measured_value=*/0.0, /*dt=*/1.0), 0.0);
}

TEST(PIDControl, DerivativeReactsToMeasurementChange) {
  PIDControl pid(
      /*k_p=*/0.0,
      /*k_i=*/0.0,
      /*k_d=*/2.0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/0.0, /*measured_value=*/10.0, /*dt=*/1.0), 20.0);
  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/0.0, /*measured_value=*/30.0, /*dt=*/2.0), 20.0);
}

TEST(PIDControl, DerivativeHandlesFractionalDt) {
  PIDControl pid(
      /*k_p=*/0.0,
      /*k_i=*/0.0,
      /*k_d=*/1.0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  pid.compute(/*setpoint=*/0.0, /*measured_value=*/0.0, /*dt=*/1.0);

  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/0.0, /*measured_value=*/7.0, /*dt=*/2.0), 3.5);
}

TEST(PIDControl, OutputClampsToUpperBound) {
  PIDControl pid(
      /*k_p=*/10.0,
      /*k_i=*/0.0,
      /*k_d=*/0.0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/-50.0,
      /*output_max=*/50.0);

  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/100.0, /*measured_value=*/0.0, /*dt=*/1.0),
      50.0);
}

TEST(PIDControl, OutputClampsToLowerBound) {
  PIDControl pid(
      /*k_p=*/10.0,
      /*k_i=*/0.0,
      /*k_d=*/0.0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/-50.0,
      /*output_max=*/50.0);

  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/0.0, /*measured_value=*/100.0, /*dt=*/1.0),
      -50.0);
}

TEST(PIDControl, OutputWithinBoundsIsUnclamped) {
  PIDControl pid(
      /*k_p=*/1.0,
      /*k_i=*/0.0,
      /*k_d=*/0.0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/-50.0,
      /*output_max=*/50.0);

  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/10.0, /*measured_value=*/0.0, /*dt=*/1.0), 10.0);
}

TEST(PIDControl, CombinedTermsAcrossMultipleSteps) {
  PIDControl pid(
      /*k_p=*/2.0,
      /*k_i=*/1.0,
      /*k_d=*/1.0,
      /*integral_min=*/-1'000.0,
      /*integral_max=*/1'000.0,
      /*output_min=*/-1'000.0,
      /*output_max=*/1'000.0);

  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/20.0, /*measured_value=*/0.0, /*dt=*/1.0), 60.0);
  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/20.0, /*measured_value=*/10.0, /*dt=*/1.0),
      60.0);
  EXPECT_DOUBLE_EQ(
      pid.compute(/*setpoint=*/20.0, /*measured_value=*/15.0, /*dt=*/2.0),
      52.5);
}

TEST(PIDControl, DtZeroProducesNonFiniteOutputInsteadOfCrashing) {
  PIDControl pid(
      /*k_p=*/1.0,
      /*k_i=*/1.0,
      /*k_d=*/1.0,
      /*integral_min=*/kWideMin,
      /*integral_max=*/kWideMax,
      /*output_min=*/kWideMin,
      /*output_max=*/kWideMax);

  EXPECT_THROW(
      pid.compute(/*setpoint=*/10.0, /*measured_value=*/0.0, /*dt=*/0.0),
      std::invalid_argument);
}
