#pragma once

#include "common/Options.hpp"
#include "sim/InitialCondition.hpp"
#include "sim/gnc/TrimTypes.hpp"

#include <string>
#include <vector>

namespace sim {
inline constexpr int SupportedScenarioSchemaVersion = 1;

enum class ScenarioCommandType { RollHold };

struct ScenarioCommand {
  ScenarioCommandType type = ScenarioCommandType::RollHold;
  double rollRad = math::DegToRad(5.0);
  bool operator==(const ScenarioCommand &) const = default;
};

struct ScenarioEventDefinition {
  double timeSec = 5.0;
  ScenarioCommand command;
  bool operator==(const ScenarioEventDefinition &) const = default;
};

struct SimScenario {
  // Contract identity
  int schemaVersion = SupportedScenarioSchemaVersion;
  std::string scenarioType = "roll_hold";
  std::string name = "Roll Hold 5deg 30s";
  std::string aircraft = std::string(opts::simulation::AircraftName);
  std::vector<std::string> controllerParameters;

  // Experiment conditions
  InitialCondition initialCondition{
      .latitudeRad = 0.0,
      .longitudeRad = 0.0,
      .altitudeAslM = math::FeetToMeters(3000.0),
      .rollRad = 0.0,
      .pitchRad = 0.0,
      .headingRad = 0.0,
      .calibratedAirspeedMps = math::KnotsToMetersPerSecond(100.0),
      .pRadPerSec = 0.0,
      .qRadPerSec = 0.0,
      .rRadPerSec = 0.0,
  };
  bool windEnabled = false;
  bool runTrim = true;
  gnc::TrimMode trimMode = gnc::TrimMode::Full;
  double durationSec = 30.0;
  double dtSec = opts::simulation::DtSec;
  std::vector<ScenarioEventDefinition> events{{}};

  // Analysis acceptance criteria, not runtime controller behavior
  double settlingBandRad = math::DegToRad(0.5);
  double settlingTimeLimitSec = 10.0;
  double overshootLimitRad = math::DegToRad(1.0);
  double maxOscillationCycles = 2.0;

  bool operator==(const SimScenario &) const = default;
};

struct ScenarioValidationError {
  std::string path;
  std::string message;
  std::string ToString() const;
};

bool ValidateSimScenario(const SimScenario &scenario,
    ScenarioValidationError *error);
bool ValidateSimScenario(const SimScenario &scenario,
    std::string *errorMessage = nullptr);
} // namespace sim
