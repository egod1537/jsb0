#pragma once

#include "gui/features/gnc/experimental/ExperimentalEvents.hpp"

namespace gui {
struct AutopilotPanelState;

class ExperimentalController {
public:
  explicit ExperimentalController(AutopilotPanelState &state);

  void Synchronize(const sim::PrimaryRollHoldConfig &config);
  void OnEvent(const PrimaryRollHoldValueChanged &event);
  void OnEvent(const ExperimentalViewStateChanged &event);

private:
  AutopilotPanelState &state_;
};
} // namespace gui
