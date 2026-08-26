#include "application/sim/gnc/hold/RollHoldController.hpp"

#include "application/sim/Aircraft.hpp"
#include "application/sim/Tick.hpp"
#include "application/sim/gnc/ControlContext.hpp"

namespace gnc {
void RollHoldController::Reset() { diagnostics_ = {}; }

bool RollHoldController::IsEnabled() const { return enabled_; }

void RollHoldController::SetEnabled(bool enabled) {
  if (enabled_ == enabled) {
    return;
  }

  enabled_ = enabled;
  if (!enabled_) {
    diagnostics_ = {};
  }
}

const RollHoldSettings &RollHoldController::GetSettings() const {
  return settings_;
}

void RollHoldController::SetSettings(const RollHoldSettings &settings) {
  settings_ = settings;
}

double RollHoldController::GetTrimAileron() const { return trimAileron_; }

void RollHoldController::SetTrimAileron(double trimAileron) {
  trimAileron_ = trimAileron;
}

const RollHoldDiagnostics &RollHoldController::GetDiagnostics() const {
  return diagnostics_;
}

std::optional<double> RollHoldController::OnTick(const sim::Aircraft &aircraft,
    const sim::Tick &, const ControlContext &context) {
  if (!enabled_) {
    return std::nullopt;
  }

  return ComputeAileronCommand(aircraft, context, settings_.targetRollRad);
}

std::optional<double> RollHoldController::OnTick(const sim::Aircraft &aircraft,
    const sim::Tick &, const ControlContext &context, double commandedRollRad) {
  return ComputeAileronCommand(aircraft, context, commandedRollRad);
}

std::optional<double> RollHoldController::ComputeAileronCommand(
    const sim::Aircraft &aircraft, const ControlContext &context,
    double targetRollRad) {
  const auto &prop = aircraft.GetProperties();
  const RollDynamics &dynamics = *context.rollDynamics;
  const double aPhi1 = dynamics.aPhi1;
  const double aPhi2 = dynamics.aPhi2;

  const double wN = settings_.naturalFrequencyRadPerSec;
  const double zeta = settings_.dampingRatio;

  const double kP = wN * wN / aPhi2;
  const double kD = (2 * zeta * wN - aPhi1) / aPhi2;

  const double error = targetRollRad - prop.Roll().Rad();
  const double newAileron =
      GetTrimAileron() + kP * error - kD * prop.P().RadPerSec();

  diagnostics_ = {
      .commandedRollRad = targetRollRad,
      .aileronCommand = newAileron,
  };
  return newAileron;
}

} // namespace gnc
