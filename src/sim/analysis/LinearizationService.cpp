#include "sim/analysis/LinearizationService.hpp"

#include "sim/Simulation.hpp"
#include "sim/control/FlightControlManager.hpp"
#include "sim/gnc/autopilot/IAutopilotAnalysis.hpp"

namespace {
gnc::IAutopilotAnalysis *FindAnalysis(sim::Simulation &simulation) {
  auto *manager = simulation.GetComponent<control::FlightControlManager>();
  return manager != nullptr
             ? dynamic_cast<gnc::IAutopilotAnalysis *>(&manager->GetAutopilot())
             : nullptr;
}

const gnc::IAutopilotAnalysis *FindAnalysis(const sim::Simulation &simulation) {
  const auto *manager =
      simulation.GetComponent<control::FlightControlManager>();
  return manager != nullptr ? dynamic_cast<const gnc::IAutopilotAnalysis *>(
                                  &manager->GetAutopilot())
                            : nullptr;
}
} // namespace

namespace sim {
bool LinearizationService::SetAutomaticUpdates(Simulation &simulation,
    bool enabled) const {
  gnc::IAutopilotAnalysis *analysis = FindAnalysis(simulation);
  if (analysis == nullptr) {
    return false;
  }
  analysis->SetAutomaticLinearizationEnabled(enabled);
  return true;
}

LinearizationServiceState LinearizationService::Capture(
    const Simulation &simulation) const {
  LinearizationServiceState state;
  const gnc::IAutopilotAnalysis *analysis = FindAnalysis(simulation);
  if (analysis == nullptr) {
    return state;
  }

  state.available = true;
  state.automaticUpdatesEnabled = analysis->IsAutomaticLinearizationEnabled();
  state.updateInProgress = analysis->IsLinearizationInProgress();
  state.errorMessage = analysis->GetLinearizationErrorMessage();
  if (const gnc::LinearizationResult *result =
          analysis->GetLinearizationResult()) {
    state.result = *result;
  }
  state.dynamicModeHistory = analysis->GetDynamicModeHistory();
  return state;
}
} // namespace sim
