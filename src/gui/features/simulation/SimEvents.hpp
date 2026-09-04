#pragma once

#include "sim/InitialCondition.hpp"
#include "sim/execution/ExecutionRequest.hpp"

namespace gui {
struct SimStartRequested {};
struct SimStopRequested {};
struct SimPlaybackToggled {};
struct SimPauseRequested {};
struct SimResumeRequested {};
struct SimResetRequested {};
struct SimStepRequested {};

struct SimRateChanged {
  double hz = 0.0;
};

struct MaximumSimulationSpeedChanged {
  bool enabled = false;
};

struct TelemetryRecordingToggled {};
struct OpenTelemetryFolderRequested {};

struct ScenarioLaunchRequested {
  sim::ExecutionRequest request;
};

enum class InitialConditionField {
  LatitudeDeg,
  LongitudeDeg,
  AltitudeAslM,
  RollDeg,
  PitchDeg,
  HeadingDeg,
  CalibratedAirspeedMps,
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
