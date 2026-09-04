#pragma once

#include "sim/gnc/control/legacy/PitchDynamics.hpp"
#include "sim/gnc/control/legacy/RollDynamics.hpp"
#include "sim/gnc/control/legacy/YawDynamics.hpp"
#include "sim/linearization/DynamicModeAnalyzer.hpp"
#include "sim/linearization/DynamicModeHistory.hpp"
#include "sim/linearization/LinearizationResult.hpp"

#include <optional>
#include <string_view>

namespace sim {
class Aircraft;
struct Tick;
} // namespace sim

namespace gnc {
class IAutopilotAnalysis {
public:
  virtual ~IAutopilotAnalysis() = default;

  // Periodic aircraft dynamics
  virtual void UpdateLinearization(sim::Aircraft &aircraft,
      const sim::Tick &tick) = 0;
  virtual bool IsAutomaticLinearizationEnabled() const = 0;
  virtual void SetAutomaticLinearizationEnabled(bool enabled) = 0;

  // Analysis results
  virtual bool IsLinearizationInProgress() const = 0;
  virtual const LinearizationResult *GetLinearizationResult() const = 0;
  virtual const DynamicModeAnalysis *GetDynamicModeAnalysis() const = 0;
  virtual const DynamicModeHistory &GetDynamicModeHistory() const = 0;
  virtual std::string_view GetLinearizationErrorMessage() const = 0;
  virtual std::optional<RollDynamics> GetRollDynamics() const = 0;
  virtual std::optional<PitchDynamics> GetPitchDynamics() const = 0;
  virtual std::optional<YawDynamics> GetYawDynamics() const = 0;
};
} // namespace gnc
