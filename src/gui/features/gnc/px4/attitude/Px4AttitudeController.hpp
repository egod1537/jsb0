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
  void Handle(const BaselineRollHoldValueChanged &event);
  void Handle(const BaselineRollHoldTuningResetRequested &event);
  void Handle(const BaselinePitchHoldTuningResetRequested &event);
  void Handle(const Px4AttitudeViewStateChanged &event);

private:
  BaselineAutopilotPanelState &baseline_;
};
} // namespace gui
