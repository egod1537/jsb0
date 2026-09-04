#pragma once

#include "gui/features/gnc/px4/attitude/Px4AttitudeEvents.hpp"
#include "gui/panels/BaselineAutopilotPanel.hpp"

namespace gui {
class Px4AttitudeController {
public:
  explicit Px4AttitudeController(BaselineAutopilotPanelState &baseline);

  // Snapshot synchronization
  void SynchronizeBaseline(const sim::BaselineRollHoldConfig &config);

  // Feature events
  void OnEvent(const BaselineRollHoldValueChanged &event);
  void OnEvent(const BaselineRollHoldTuningResetRequested &event);
  void OnEvent(const BaselinePitchHoldTuningResetRequested &event);
  void OnEvent(const Px4AttitudeViewStateChanged &event);

private:
  BaselineAutopilotPanelState &baseline_;
};
} // namespace gui
