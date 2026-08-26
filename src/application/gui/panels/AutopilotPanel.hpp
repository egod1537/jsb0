#pragma once

#include <functional>

namespace gui {
struct AutopilotPanelState {
  // Autopilot control selection
  bool rollHold = false;

  // Roll Hold target
  double rollTargetDeg = 0.0;

  // Desired response
  double rollHoldDampingRatio = 0.7;
  double rollHoldNaturalFrequencyRadPerSec = 1.0;
  bool rollHoldResponseOpen = true;
};

struct AutopilotPanelProps {
  AutopilotPanelState &state;

  // Roll telemetry and actions
  double currentRollDeg = 0.0;
  double currentRollRateDegPerSec = 0.0;
  double currentAileron = 0.0;
  bool rollHoldActive = false;
  bool rollHoldPreparing = false;
  std::function<void()> captureCurrentRoll;
};

class AutopilotPanel {
public:
  static void Draw(AutopilotPanelState &state);
  static void Draw(const AutopilotPanelProps &props);
};
} // namespace gui
