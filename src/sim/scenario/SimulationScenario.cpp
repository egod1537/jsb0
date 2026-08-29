#include "sim/scenario/SimulationScenario.hpp"

#include <cmath>

namespace {
bool ValidationFailed(std::string *errorMessage, const char *message) {
  if (errorMessage != nullptr) {
    *errorMessage = message;
  }

  return false;
}

bool IsSupportedTrimMode(gnc::TrimMode mode) {
  switch (mode) {
  case gnc::TrimMode::Longitudinal:
  case gnc::TrimMode::Full:
  case gnc::TrimMode::Ground:
    return true;
  }

  return false;
}
} // namespace

namespace sim {
bool ValidateSimulationScenario(const SimulationScenario &scenario,
    std::string *errorMessage) {
  if (errorMessage != nullptr) {
    errorMessage->clear();
  }

  if (scenario.name.empty()) {
    return ValidationFailed(errorMessage, "name must not be empty");
  }
  if (scenario.schemaVersion != 1) {
    return ValidationFailed(errorMessage, "schema_version must be 1");
  }
  if (scenario.scenarioType != "roll_hold") {
    return ValidationFailed(errorMessage, "scenario_type must be roll_hold");
  }
  if (scenario.aircraft != "c172x") {
    return ValidationFailed(errorMessage, "aircraft must be c172x");
  }
  if (scenario.autopilot != "primary" && scenario.autopilot != "baseline") {
    return ValidationFailed(errorMessage,
        "autopilot must be primary or baseline");
  }
  if (!IsSupportedTrimMode(scenario.trimMode)) {
    return ValidationFailed(errorMessage, "trim.mode has an unsupported value");
  }
  if (!std::isfinite(scenario.altitudeFt)) {
    return ValidationFailed(errorMessage,
        "initial_condition.altitude_ft must be finite");
  }
  if (!std::isfinite(scenario.airspeedKts) || scenario.airspeedKts < 0.0) {
    return ValidationFailed(errorMessage,
        "initial_condition.airspeed_kts must be finite and non-negative");
  }
  if (!std::isfinite(scenario.initialRollDeg)) {
    return ValidationFailed(errorMessage,
        "initial_condition.roll_deg must be finite");
  }
  if (!std::isfinite(scenario.initialPitchDeg)) {
    return ValidationFailed(errorMessage,
        "initial_condition.pitch_deg must be finite");
  }
  if (!std::isfinite(scenario.initialHeadingDeg)) {
    return ValidationFailed(errorMessage,
        "initial_condition.heading_deg must be finite");
  }
  if (!std::isfinite(scenario.commandedRollDeg)) {
    return ValidationFailed(errorMessage, "command.roll_deg must be finite");
  }
  if (!std::isfinite(scenario.commandStartSec)
      || scenario.commandStartSec < 0.0) {
    return ValidationFailed(errorMessage,
        "command.start_sec must be finite and non-negative");
  }
  if (!std::isfinite(scenario.durationSec) || scenario.durationSec <= 0.0) {
    return ValidationFailed(errorMessage,
        "simulation.duration_sec must be finite and greater than 0");
  }
  if (scenario.commandStartSec > scenario.durationSec) {
    return ValidationFailed(errorMessage,
        "command.start_sec must not exceed simulation.duration_sec");
  }
  if (!std::isfinite(scenario.settlingBandDeg)
      || scenario.settlingBandDeg < 0.0) {
    return ValidationFailed(errorMessage,
        "acceptance.settling_band_deg must be finite and non-negative");
  }
  if (!std::isfinite(scenario.settlingTimeLimitSec)
      || scenario.settlingTimeLimitSec < 0.0) {
    return ValidationFailed(errorMessage,
        "acceptance.settling_time_limit_sec must be finite and non-negative");
  }
  if (!std::isfinite(scenario.overshootLimitDeg)
      || scenario.overshootLimitDeg < 0.0) {
    return ValidationFailed(errorMessage,
        "acceptance.overshoot_limit_deg must be finite and non-negative");
  }
  if (!std::isfinite(scenario.maxOscillationCycles)
      || scenario.maxOscillationCycles < 0.0) {
    return ValidationFailed(errorMessage,
        "acceptance.max_oscillation_cycles must be finite and non-negative");
  }

  return true;
}
} // namespace sim
