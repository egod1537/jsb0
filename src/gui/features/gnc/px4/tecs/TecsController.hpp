#pragma once

#include "gui/features/gnc/px4/tecs/TecsEvents.hpp"

namespace sim {
struct BaselineRollHoldConfig;
}

namespace gui {
struct BaselineAutopilotPanelState;

class TecsController {
public:
  explicit TecsController(BaselineAutopilotPanelState &state);

  void Synchronize(const sim::BaselineRollHoldConfig &config);

  void Handle(const BaselineTecsValueChanged &event);
  void Handle(const BaselineTecsParameterChanged &event);
  void Handle(const BaselineTecsTuningResetRequested &event);
  void Handle(const BaselineTecsAltitudeCaptureRequested &event);
  void Handle(const BaselineTecsAirspeedCaptureRequested &event);

private:
  BaselineAutopilotPanelState &state_;
};
} // namespace gui
