#pragma once

#include "sim/InitialCondition.hpp"
#include "sim/scenario/SimulationScenario.hpp"

namespace gui {
struct SimulationStartRequested {};
struct SimulationStopRequested {};
struct SimulationPauseRequested {};
struct SimulationResumeRequested {};
struct SimulationResetRequested {};
struct SimulationStepRequested {};

struct SimulationRateChanged {
  double hz = 0.0;
};

struct MaximumSimulationSpeedChanged {
  bool enabled = false;
};

struct TelemetryRecordingToggled {};
struct OpenTelemetryFolderRequested {};

struct ScenarioLaunchRequested {
  sim::SimulationScenario scenario;
};

enum class InitialConditionField {
  LatitudeDeg,
  LongitudeDeg,
  AltitudeFt,
  RollDeg,
  PitchDeg,
  HeadingDeg,
  AirspeedKts,
};

struct InitialConditionFieldChanged {
  InitialConditionField field = InitialConditionField::LatitudeDeg;
  double value = 0.0;
};

struct UseCurrentInitialConditionRequested {
  sim::InitialCondition current;
};

struct RestoreDefaultInitialConditionRequested {
  sim::InitialCondition defaults;
};

struct ResetWithInitialConditionRequested {};
} // namespace gui
