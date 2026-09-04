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

  void OnEvent(const BaselineTecsValueChanged &event);
  void OnEvent(const BaselineTecsParameterChanged &event);
  void OnEvent(const BaselineTecsTuningResetRequested &event);
  void OnEvent(const BaselineTecsAltitudeCaptureRequested &event);
  void OnEvent(const BaselineTecsAirspeedCaptureRequested &event);

private:
  BaselineAutopilotPanelState &state_;
};
} // namespace gui
