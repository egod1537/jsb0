#include "sim/runtime/SimInstanceSet.hpp"

#include "sim/Simulation.hpp"
#include "sim/control/FlightControlManager.hpp"

#include <utility>

namespace sim {
namespace {
std::string GetSimulationError(const Simulation &simulation,
    std::string fallback) {
  return simulation.GetErrorTracker().GetLastError().value_or(
      std::move(fallback));
}
} // namespace

SimInstanceSet::SimInstanceSet(Simulation &primary,
    Simulation *baseline)
    : primary_(primary), baseline_(baseline) {}

bool SimInstanceSet::Initialize(std::string_view aircraftName,
    double simulationHz, std::string &error) const {
  if (!primary_.Initialize(aircraftName, simulationHz)) {
    error = GetSimulationError(primary_,
        "Failed to initialize primary simulation.");
    return false;
  }
  if (baseline_ != nullptr
      && !baseline_->Initialize(aircraftName, simulationHz)) {
    error = GetSimulationError(*baseline_,
        "Failed to initialize baseline simulation.");
    primary_.Shutdown();
    return false;
  }
  error.clear();
  return true;
}

void SimInstanceSet::Shutdown() const {
  if (baseline_ != nullptr) {
    baseline_->Shutdown();
  }
  primary_.Shutdown();
}

bool SimInstanceSet::Reset(const InitialCondition *initialCondition,
    std::string &error) const {
  const auto reset = [initialCondition](Simulation &simulation) {
    return initialCondition != nullptr ? simulation.Reset(*initialCondition)
                                       : simulation.Reset();
  };
  if (!reset(primary_)) {
    error = GetSimulationError(primary_, "Failed to reset primary simulation.");
    return false;
  }
  if (baseline_ != nullptr && !reset(*baseline_)) {
    error =
        GetSimulationError(*baseline_, "Failed to reset baseline simulation.");
    return false;
  }
  error.clear();
  return true;
}

bool SimInstanceSet::Step(double dtSec, std::string &error) const {
  if (!SynchronizeControlState(error)) {
    return false;
  }
  if (!primary_.Step(dtSec)) {
    error = GetSimulationError(primary_, "Primary simulation step failed.");
    return false;
  }
  if (baseline_ != nullptr && !baseline_->Step(dtSec)) {
    error = GetSimulationError(*baseline_, "Baseline simulation step failed.");
    return false;
  }
  error.clear();
  return true;
}

bool SimInstanceSet::SynchronizeControlState(std::string &error) const {
  if (baseline_ == nullptr) {
    return true;
  }
  const auto &primaryManager = primary_.GetFlightControlManager();
  auto &baselineManager = baseline_->GetFlightControlManager();
  baselineManager.GetManualController().SetCommandedInput(
      primaryManager.GetManualController().GetCommandedInput());
  return true;
}
} // namespace sim
