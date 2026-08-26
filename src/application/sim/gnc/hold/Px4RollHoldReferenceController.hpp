#pragma once

#include "application/sim/gnc/Controller.hpp"

#include <optional>

namespace sim {
class Aircraft;
struct Tick;
} // namespace sim

namespace gnc {
struct Px4RollHoldReferenceSettings {
  double timeConstantSec = 0.35;
  double maximumRollRateRadPerSec = 1.2217304763960306;
  double rateProportionalGain = 0.160;
  double rateIntegralGain = 0.080;
  double rateDerivativeGain = 0.0;
  double rateFeedForwardGain = 0.80;
  double integratorLimit = 0.15;
  double trimAirspeedMps = 15.0;
  double stallAirspeedMps = 7.0;
  double trimRollCommand = 0.0;
};

struct Px4RollHoldReferenceDiagnostics {
  double targetRollRad = 0.0;
  double rollErrorRad = 0.0;
  double bodyRateSetpointRadPerSec = 0.0;
  double bodyRateErrorRadPerSec = 0.0;
  double rateIntegrator = 0.0;
  double airspeedScaling = 1.0;
  double rollTorqueCommand = 0.0;
  double aileronCommand = 0.0;
};

class Px4RollHoldReferenceController final : public Controller {
public:
  // Lifecycle
  void Reset() override;

  // Baseline Roll Hold execution
  bool IsEnabled() const;
  void SetEnabled(bool enabled);

  // PX4 parameter snapshot
  const Px4RollHoldReferenceSettings &GetSettings() const;
  void SetSettings(const Px4RollHoldReferenceSettings &settings);

  // Baseline output and diagnostics
  std::optional<double> OnTick(const sim::Aircraft &aircraft,
      const sim::Tick &tick, double targetRollRad);
  const Px4RollHoldReferenceDiagnostics &GetDiagnostics() const;

private:
  void UpdateIntegrator(double rateErrorRadPerSec, double dtSec,
      bool positiveSaturation, bool negativeSaturation);

  // Mode and PX4 parameter snapshot
  bool enabled_ = false;
  Px4RollHoldReferenceSettings settings_;

  // Controller state and last result
  double rateIntegrator_ = 0.0;
  Px4RollHoldReferenceDiagnostics diagnostics_;
};
} // namespace gnc
