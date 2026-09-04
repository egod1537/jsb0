#pragma once

#include "sim/runtime/SimContracts.hpp"

namespace gui {
struct BaselineRollHoldConfigChanged {
  sim::BaselineRollHoldConfig config;
};

enum class BaselineRollHoldField {
  Enabled,
  TargetDeg,
  PitchHoldEnabled,
  TargetPitchDeg,
  PitchTimeConstantSec,
  MaximumPositivePitchRateDegPerSec,
  MaximumNegativePitchRateDegPerSec,
  PitchRateProportionalGain,
  PitchRateIntegralGain,
  PitchRateDerivativeGain,
  PitchRateFeedForwardGain,
  PitchIntegratorLimit,
  CourseHoldEnabled,
  TargetCourseDeg,
  CourseGuidancePeriodSec,
  CourseGuidanceDampingRatio,
  CourseMaximumRollDeg,
  CourseMaximumRollSetpointRateDegPerSec,
  TimeConstantSec,
  MaximumRateDegPerSec,
  RateProportionalGain,
  RateIntegralGain,
  RateDerivativeGain,
  RateFeedForwardGain,
  IntegratorLimit,
  DirectRollRateTestEnabled,
  DirectRollRateCommandDegPerSec,
  YawRateControlEnabled,
  CoordinatedTurnEnabled,
  MaximumYawRateDegPerSec,
  YawRateProportionalGain,
  YawRateIntegralGain,
  YawRateDerivativeGain,
  YawRateFeedForwardGain,
  YawIntegratorLimit,
  SideslipToYawRateGain,
  YawRateWashoutTimeConstantSec,
  RollToYawFeedForwardGain,
};

struct BaselineRollHoldValueChanged {
  BaselineRollHoldField field = BaselineRollHoldField::Enabled;
  double value = 0.0;
};

struct BaselineRollHoldTuningResetRequested {};
struct BaselinePitchHoldTuningResetRequested {};

struct Px4AttitudeViewStateChanged {
  bool rollTuningOpen = false;
  bool rollDiagnosticsOpen = true;
  bool pitchTuningOpen = false;
  bool pitchDiagnosticsOpen = true;
};
} // namespace gui
