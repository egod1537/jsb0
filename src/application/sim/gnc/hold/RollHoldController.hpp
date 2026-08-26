#pragma once

#include "application/sim/gnc/Controller.hpp"
#include "application/sim/gnc/hold/RollHoldSettings.hpp"

#include <optional>

namespace sim {
class Aircraft;
struct Tick;
} // namespace sim

namespace gnc {
struct ControlContext;

struct RollHoldDiagnostics {
  double commandedRollRad = 0.0;
  double aileronCommand = 0.0;
};

class RollHoldController final : public Controller {
public:
  // Lifecycle
  void Reset() override;

  // Mode
  bool IsEnabled() const;
  void SetEnabled(bool enabled);

  // Configuration
  const RollHoldSettings &GetSettings() const;
  void SetSettings(const RollHoldSettings &settings);

  // Trim reference
  double GetTrimAileron() const;
  void SetTrimAileron(double trimAileron);

  // Diagnostics
  const RollHoldDiagnostics &GetDiagnostics() const;

  // Standalone control output
  std::optional<double> OnTick(const sim::Aircraft &aircraft,
      const sim::Tick &tick, const ControlContext &context);

  // Cascaded control output
  std::optional<double> OnTick(const sim::Aircraft &aircraft,
      const sim::Tick &tick, const ControlContext &context,
      double commandedRollRad);

private:
  std::optional<double> ComputeAileronCommand(const sim::Aircraft &aircraft,
      const ControlContext &context, double targetRollRad);

  // Mode and configuration
  bool enabled_ = false;
  RollHoldSettings settings_;

  // Trim reference
  double trimAileron_ = 0.0;

  // Last control result
  RollHoldDiagnostics diagnostics_;
};
} // namespace gnc
