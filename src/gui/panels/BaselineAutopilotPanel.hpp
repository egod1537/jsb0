#pragma once

#include "gui/architecture/EventSink.hpp"
#include "gui/features/gnc/px4/attitude/Px4AttitudeEvents.hpp"
#include "gui/features/gnc/px4/tecs/TecsEvents.hpp"
#include "sim/gnc/config/Px4ControlProfile.hpp"
#include "sim/gnc/control/attitude/Px4PitchParameterMetadata.hpp"
#include "sim/gnc/control/attitude/Px4RollParameterMetadata.hpp"
#include "sim/gnc/control/lateral/Px4CourseParameterMetadata.hpp"
#include "sim/gnc/control/yaw/Px4YawRateParameterMetadata.hpp"
#include "sim/gnc/tecs/Px4TecsController.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace gui {
struct BaselineAutopilotPanelState {
  BaselineAutopilotPanelState();

  // Baseline Roll Hold
  bool rollHold = false;
  double rollTargetDeg = 0.0;

  // Baseline Pitch Hold
  bool pitchHold = false;
  double pitchTargetDeg = 0.0;

  // PX4 TECS longitudinal outer loop
  bool tecs = false;
  double tecsTargetAltitudeM = 304.8;
  double tecsTargetAirspeedMps = 41.1556;
  gnc::Px4TecsSettings tecsSettings;

  // PX4 v1.17 Pitch Hold tuning
  double px4PitchTimeConstantSec = 0.0;
  double px4PitchMaximumPositiveRateDegPerSec = 0.0;
  double px4PitchMaximumNegativeRateDegPerSec = 0.0;
  double px4PitchRateProportionalGain = 0.0;
  double px4PitchRateIntegralGain = 0.0;
  double px4PitchRateDerivativeGain = 0.0;
  double px4PitchRateFeedForwardGain = 0.0;
  double px4PitchIntegratorLimit = 0.0;

  // PX4 course/lateral outer loop
  bool courseHold = false;
  double targetCourseDeg = 0.0;
  double courseGuidancePeriodSec = 0.0;
  double courseGuidanceDampingRatio = 0.0;
  double courseMaximumRollDeg = 0.0;
  double courseMaximumRollSetpointRateDegPerSec = 0.0;

  // PX4 v1.17 Roll Hold tuning
  double px4RollTimeConstantSec = 0.0;
  double px4RollMaximumRateDegPerSec = 0.0;
  double px4RollRateProportionalGain = 0.0;
  double px4RollRateIntegralGain = 0.0;
  double px4RollRateDerivativeGain = 0.0;
  double px4RollRateFeedForwardGain = 0.0;
  double px4RollIntegratorLimit = 0.0;

  // Temporary direct-rate tuning bypass
  bool directRollRateTestEnabled = false;
  double directRollRateCommandDegPerSec = 0.0;

  // Experimental yaw coordination and Dutch-roll damping
  bool yawRateControlEnabled = false;
  bool coordinatedTurnEnabled = true;
  double px4YawMaximumRateDegPerSec = 0.0;
  double px4YawRateProportionalGain = 0.8;
  double px4YawRateIntegralGain = 0.0;
  double px4YawRateDerivativeGain = 0.0;
  double px4YawRateFeedForwardGain = 0.0;
  double px4YawIntegratorLimit = 0.0;
  double sideslipToYawRateGain = 8.0;
  double yawRateWashoutTimeConstantSec = 0.0;
  double rollToYawFeedForwardGain = 0.0;

  // Foldout state
  bool px4RollTuningOpen = false;
  bool px4RollDiagnosticsOpen = true;
  bool px4PitchTuningOpen = false;
  bool px4PitchDiagnosticsOpen = true;
};

struct BaselinePx4RollHoldParameterBinding {
  BaselineRollHoldField field;
  gnc::Px4RollHoldParameter parameter;
  double BaselineAutopilotPanelState::*value;
};

struct BaselinePx4CourseHoldParameterBinding {
  BaselineRollHoldField field;
  gnc::Px4CourseHoldParameter parameter;
  double BaselineAutopilotPanelState::*value;
};

struct BaselinePx4PitchHoldParameterBinding {
  BaselineRollHoldField field;
  gnc::Px4PitchHoldParameter parameter;
  double BaselineAutopilotPanelState::*value;
};

struct BaselinePx4YawRateParameterBinding {
  BaselineRollHoldField field;
  gnc::Px4YawRateParameter parameter;
  double BaselineAutopilotPanelState::*value;
};

inline constexpr std::array<BaselinePx4YawRateParameterBinding,
    static_cast<std::size_t>(gnc::Px4YawRateParameter::Count)>
    BaselinePx4YawRateParameterBindings{{
        {BaselineRollHoldField::MaximumYawRateDegPerSec,
            gnc::Px4YawRateParameter::MaximumYawRate,
            &BaselineAutopilotPanelState::px4YawMaximumRateDegPerSec},
        {BaselineRollHoldField::YawRateProportionalGain,
            gnc::Px4YawRateParameter::RateProportionalGain,
            &BaselineAutopilotPanelState::px4YawRateProportionalGain},
        {BaselineRollHoldField::YawRateIntegralGain,
            gnc::Px4YawRateParameter::RateIntegralGain,
            &BaselineAutopilotPanelState::px4YawRateIntegralGain},
        {BaselineRollHoldField::YawRateDerivativeGain,
            gnc::Px4YawRateParameter::RateDerivativeGain,
            &BaselineAutopilotPanelState::px4YawRateDerivativeGain},
        {BaselineRollHoldField::YawRateFeedForwardGain,
            gnc::Px4YawRateParameter::RateFeedForwardGain,
            &BaselineAutopilotPanelState::px4YawRateFeedForwardGain},
        {BaselineRollHoldField::YawIntegratorLimit,
            gnc::Px4YawRateParameter::IntegratorLimit,
            &BaselineAutopilotPanelState::px4YawIntegratorLimit},
        {BaselineRollHoldField::RollToYawFeedForwardGain,
            gnc::Px4YawRateParameter::RollToYawFeedForwardGain,
            &BaselineAutopilotPanelState::rollToYawFeedForwardGain},
        {BaselineRollHoldField::SideslipToYawRateGain,
            gnc::Px4YawRateParameter::SideslipToYawRateGain,
            &BaselineAutopilotPanelState::sideslipToYawRateGain},
        {BaselineRollHoldField::YawRateWashoutTimeConstantSec,
            gnc::Px4YawRateParameter::YawRateWashoutTimeConstant,
            &BaselineAutopilotPanelState::yawRateWashoutTimeConstantSec},
    }};

inline constexpr std::array<BaselinePx4PitchHoldParameterBinding, 8>
    BaselinePx4PitchHoldParameterBindings{{
        {BaselineRollHoldField::PitchTimeConstantSec,
            gnc::Px4PitchHoldParameter::TimeConstant,
            &BaselineAutopilotPanelState::px4PitchTimeConstantSec},
        {BaselineRollHoldField::MaximumPositivePitchRateDegPerSec,
            gnc::Px4PitchHoldParameter::MaximumPositivePitchRate,
            &BaselineAutopilotPanelState::px4PitchMaximumPositiveRateDegPerSec},
        {BaselineRollHoldField::MaximumNegativePitchRateDegPerSec,
            gnc::Px4PitchHoldParameter::MaximumNegativePitchRate,
            &BaselineAutopilotPanelState::px4PitchMaximumNegativeRateDegPerSec},
        {BaselineRollHoldField::PitchRateProportionalGain,
            gnc::Px4PitchHoldParameter::RateProportionalGain,
            &BaselineAutopilotPanelState::px4PitchRateProportionalGain},
        {BaselineRollHoldField::PitchRateIntegralGain,
            gnc::Px4PitchHoldParameter::RateIntegralGain,
            &BaselineAutopilotPanelState::px4PitchRateIntegralGain},
        {BaselineRollHoldField::PitchRateDerivativeGain,
            gnc::Px4PitchHoldParameter::RateDerivativeGain,
            &BaselineAutopilotPanelState::px4PitchRateDerivativeGain},
        {BaselineRollHoldField::PitchRateFeedForwardGain,
            gnc::Px4PitchHoldParameter::RateFeedForwardGain,
            &BaselineAutopilotPanelState::px4PitchRateFeedForwardGain},
        {BaselineRollHoldField::PitchIntegratorLimit,
            gnc::Px4PitchHoldParameter::IntegratorLimit,
            &BaselineAutopilotPanelState::px4PitchIntegratorLimit},
    }};

inline constexpr std::array<BaselinePx4CourseHoldParameterBinding, 4>
    BaselinePx4CourseHoldParameterBindings{{
        {BaselineRollHoldField::CourseGuidancePeriodSec,
            gnc::Px4CourseHoldParameter::GuidancePeriod,
            &BaselineAutopilotPanelState::courseGuidancePeriodSec},
        {BaselineRollHoldField::CourseGuidanceDampingRatio,
            gnc::Px4CourseHoldParameter::GuidanceDamping,
            &BaselineAutopilotPanelState::courseGuidanceDampingRatio},
        {BaselineRollHoldField::CourseMaximumRollDeg,
            gnc::Px4CourseHoldParameter::MaximumRoll,
            &BaselineAutopilotPanelState::courseMaximumRollDeg},
        {BaselineRollHoldField::CourseMaximumRollSetpointRateDegPerSec,
            gnc::Px4CourseHoldParameter::MaximumRollSetpointRate,
            &BaselineAutopilotPanelState::
                courseMaximumRollSetpointRateDegPerSec},
    }};

inline constexpr std::array<BaselinePx4RollHoldParameterBinding, 7>
    BaselinePx4RollHoldParameterBindings{{
        {BaselineRollHoldField::TimeConstantSec,
            gnc::Px4RollHoldParameter::TimeConstant,
            &BaselineAutopilotPanelState::px4RollTimeConstantSec},
        {BaselineRollHoldField::MaximumRateDegPerSec,
            gnc::Px4RollHoldParameter::MaximumRollRate,
            &BaselineAutopilotPanelState::px4RollMaximumRateDegPerSec},
        {BaselineRollHoldField::RateProportionalGain,
            gnc::Px4RollHoldParameter::RateProportionalGain,
            &BaselineAutopilotPanelState::px4RollRateProportionalGain},
        {BaselineRollHoldField::RateIntegralGain,
            gnc::Px4RollHoldParameter::RateIntegralGain,
            &BaselineAutopilotPanelState::px4RollRateIntegralGain},
        {BaselineRollHoldField::RateDerivativeGain,
            gnc::Px4RollHoldParameter::RateDerivativeGain,
            &BaselineAutopilotPanelState::px4RollRateDerivativeGain},
        {BaselineRollHoldField::RateFeedForwardGain,
            gnc::Px4RollHoldParameter::RateFeedForwardGain,
            &BaselineAutopilotPanelState::px4RollRateFeedForwardGain},
        {BaselineRollHoldField::IntegratorLimit,
            gnc::Px4RollHoldParameter::IntegratorLimit,
            &BaselineAutopilotPanelState::px4RollIntegratorLimit},
    }};

template <typename Binding, std::size_t Size, typename MetadataLookup>
inline bool SetBaselinePx4Parameter(BaselineAutopilotPanelState &state,
    BaselineRollHoldField field, double value,
    const std::array<Binding, Size> &bindings, MetadataLookup metadataLookup,
    bool roundToUiPrecision) {
  if (!std::isfinite(value)) {
    return false;
  }
  for (const Binding &binding : bindings) {
    if (binding.field != field) {
      continue;
    }
    const auto &metadata = metadataLookup(binding.parameter);
    double clamped = std::clamp(value, metadata.minimum, metadata.maximum);
    if (roundToUiPrecision) {
      constexpr double PrecisionScale = 1000.0;
      clamped = std::round(clamped * PrecisionScale) / PrecisionScale;
    }
    state.*(binding.value) =
        std::clamp(clamped, metadata.minimum, metadata.maximum);
    return true;
  }
  return false;
}

inline void ResetBaselinePx4RollHoldTuning(BaselineAutopilotPanelState &state) {
  const gnc::Px4ControlProfile &profile = gnc::GetC172xPx4ControlProfile();
  for (const BaselinePx4RollHoldParameterBinding &binding :
      BaselinePx4RollHoldParameterBindings) {
    state.*(binding.value) =
        gnc::GetPx4RollHoldParameterValue(profile.roll, binding.parameter);
  }
  for (const BaselinePx4YawRateParameterBinding &binding :
      BaselinePx4YawRateParameterBindings) {
    state.*(binding.value) =
        gnc::GetPx4YawRateParameterValue(profile.yaw, binding.parameter);
  }
  state.directRollRateTestEnabled = false;
  state.directRollRateCommandDegPerSec = 0.0;
  state.yawRateControlEnabled = false;
  state.coordinatedTurnEnabled =
      profile.yaw.setpointMode == gnc::Px4YawRateSetpointMode::CoordinatedTurn;
}

inline void ResetBaselinePx4CourseHoldTuning(
    BaselineAutopilotPanelState &state) {
  for (const BaselinePx4CourseHoldParameterBinding &binding :
      BaselinePx4CourseHoldParameterBindings) {
    state.*(binding.value) = gnc::GetPx4CourseHoldParameterValue(
        gnc::GetC172xPx4ControlProfile().course,
        binding.parameter);
  }
}

inline void ResetBaselinePx4PitchHoldTuning(
    BaselineAutopilotPanelState &state) {
  for (const BaselinePx4PitchHoldParameterBinding &binding :
      BaselinePx4PitchHoldParameterBindings) {
    state.*(binding.value) = gnc::GetPx4PitchHoldParameterValue(
        gnc::GetC172xPx4ControlProfile().pitch,
        binding.parameter);
  }
}

inline bool SetBaselinePx4PitchHoldParameter(BaselineAutopilotPanelState &state,
    BaselineRollHoldField field, double value) {
  return SetBaselinePx4Parameter(state,
      field,
      value,
      BaselinePx4PitchHoldParameterBindings,
      gnc::GetPx4PitchHoldParameterMetadata,
      true);
}

inline bool SetBaselinePx4CourseHoldParameter(
    BaselineAutopilotPanelState &state, BaselineRollHoldField field,
    double value) {
  return SetBaselinePx4Parameter(state,
      field,
      value,
      BaselinePx4CourseHoldParameterBindings,
      gnc::GetPx4CourseHoldParameterMetadata,
      false);
}

inline bool SetBaselinePx4RollHoldParameter(BaselineAutopilotPanelState &state,
    BaselineRollHoldField field, double value) {
  return SetBaselinePx4Parameter(state,
      field,
      value,
      BaselinePx4RollHoldParameterBindings,
      gnc::GetPx4RollHoldParameterMetadata,
      true);
}

inline bool SetBaselinePx4YawRateParameter(BaselineAutopilotPanelState &state,
    BaselineRollHoldField field, double value) {
  return SetBaselinePx4Parameter(state,
      field,
      value,
      BaselinePx4YawRateParameterBindings,
      gnc::GetPx4YawRateParameterMetadata,
      false);
}

inline BaselineAutopilotPanelState::BaselineAutopilotPanelState() {
  ResetBaselinePx4RollHoldTuning(*this);
  ResetBaselinePx4PitchHoldTuning(*this);
  ResetBaselinePx4CourseHoldTuning(*this);
}

struct BaselineAutopilotPanelProps {
  BaselineAutopilotPanelState &state;
  double currentRollDeg = 0.0;
  double currentPitchDeg = 0.0;
  double currentAltitudeAglM = 0.0;
  double currentCalibratedAirspeedMps = 0.0;
  double currentCourseDeg = 0.0;
  double currentRollRateDegPerSec = 0.0;
  double currentPitchRateDegPerSec = 0.0;
  double currentAileron = 0.0;
  double currentElevator = 0.0;
  bool rollHoldActive = false;
  bool pitchHoldActive = false;
  bool tecsActive = false;
  bool courseHoldActive = false;
  architecture::EventSink<BaselineRollHoldValueChanged> valueEvents;
  architecture::EventSink<BaselineRollHoldTuningResetRequested> resetEvents;
  architecture::EventSink<BaselinePitchHoldTuningResetRequested>
      pitchResetEvents;
  architecture::EventSink<BaselineTecsValueChanged> tecsValueEvents;
  architecture::EventSink<BaselineTecsParameterChanged> tecsParameterEvents;
  architecture::EventSink<BaselineTecsTuningResetRequested> tecsResetEvents;
  architecture::EventSink<BaselineTecsAltitudeCaptureRequested>
      tecsAltitudeCaptureEvents;
  architecture::EventSink<BaselineTecsAirspeedCaptureRequested>
      tecsAirspeedCaptureEvents;
  double px4RollAileronCommand = 0.0;
  double px4RollRateSetpointDegPerSec = 0.0;
  double px4RollErrorDeg = 0.0;
  double px4AirspeedScaling = 1.0;
  double px4PitchElevatorCommand = 0.0;
  double px4PitchRateSetpointDegPerSec = 0.0;
  double px4PitchErrorDeg = 0.0;
  double px4PitchAirspeedScaling = 1.0;
  double tecsInternalAltitudeSetpointM = 0.0;
  double tecsTargetPitchDeg = 0.0;
  double tecsTargetThrottle = 0.0;
  double tecsTotalEnergyError = 0.0;
  double tecsEnergyBalanceError = 0.0;
  bool tecsUnderspeedProtectionActive = false;
  bool tecsOverspeedProtectionActive = false;
  double courseErrorDeg = 0.0;
  double courseRawRollSetpointDeg = 0.0;
  double courseLimitedRollSetpointDeg = 0.0;
};

class BaselineAutopilotPanel {
public:
  static void Draw(const BaselineAutopilotPanelProps &props);
};
} // namespace gui
