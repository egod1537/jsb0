#pragma once

#include "gui/features/gnc/experimental/ExperimentalEvents.hpp"

namespace gui {
struct AutopilotPanelState;

class ExperimentalController {
public:
  explicit ExperimentalController(AutopilotPanelState &state);

  void Synchronize(const sim::PrimaryRollHoldConfig &config);
  void Handle(const PrimaryRollHoldValueChanged &event);
  void Handle(const ExperimentalViewStateChanged &event);

private:
  AutopilotPanelState &state_;
};
} // namespace gui
