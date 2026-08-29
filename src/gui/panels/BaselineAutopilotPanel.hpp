#pragma once

#include "gui/architecture/EventSink.hpp"
#include "gui/features/gnc/GNCEvents.hpp"

namespace gui {
struct BaselineAutopilotPanelState {
  // Baseline Roll Hold
  bool rollHold = false;
  double rollTargetDeg = 0.0;

  // PX4 v1.17 Roll Hold tuning
  double px4RollTimeConstantSec = 0.35;
  double px4RollMaximumRateDegPerSec = 70.0;
  double px4RollRateProportionalGain = 0.160;
  double px4RollRateIntegralGain = 0.080;
  double px4RollRateDerivativeGain = 0.0;
  double px4RollRateFeedForwardGain = 0.80;
  double px4RollIntegratorLimit = 0.15;

  // Foldout state
  bool px4RollTuningOpen = false;
  bool px4RollDiagnosticsOpen = true;
};

struct BaselineAutopilotPanelProps {
  BaselineAutopilotPanelState &state;
  double currentRollDeg = 0.0;
  double currentRollRateDegPerSec = 0.0;
  double currentAileron = 0.0;
  bool rollHoldActive = false;
  architecture::EventSink<BaselineRollHoldValueChanged> valueEvents;
  architecture::EventSink<BaselineRollHoldTuningResetRequested> resetEvents;
  double px4RollAileronCommand = 0.0;
  double px4RollRateSetpointDegPerSec = 0.0;
  double px4RollErrorDeg = 0.0;
  double px4AirspeedScaling = 1.0;
};

class BaselineAutopilotPanel {
public:
  static void Draw(const BaselineAutopilotPanelProps &props);
};
} // namespace gui
