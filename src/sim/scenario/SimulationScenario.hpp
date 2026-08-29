#pragma once

#include "sim/gnc/TrimTypes.hpp"

#include <string>

namespace sim {
struct SimulationScenario {
  // Contract identity and capability
  int schemaVersion = 1;
  std::string scenarioType = "roll_hold";
  std::string name = "C172 Roll Hold 5deg";
  std::string aircraft = "c172x";
  std::string autopilot = "primary";
  std::string sourceFile;

  // Initial condition
  double altitudeFt = 3000.0;
  double airspeedKts = 100.0;
  double initialRollDeg = 0.0;
  double initialPitchDeg = 0.0;
  double initialHeadingDeg = 0.0;

  // Environment
  bool windEnabled = false;

  // Trim
  bool runTrim = true;
  gnc::TrimMode trimMode = gnc::TrimMode::Full;

  // Simulation and command
  double durationSec = 30.0;
  double commandStartSec = 5.0;
  double commandedRollDeg = 5.0;

  // Acceptance criteria
  double settlingBandDeg = 0.5;
  double settlingTimeLimitSec = 10.0;
  double overshootLimitDeg = 1.0;
  double maxOscillationCycles = 2.0;

  bool operator==(const SimulationScenario &) const = default;
};

bool ValidateSimulationScenario(const SimulationScenario &scenario,
    std::string *errorMessage = nullptr);
} // namespace sim
