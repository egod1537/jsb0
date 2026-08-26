#pragma once

#include "application/gui/panels/AutopilotSelection.hpp"
#include "application/gui/panels/AutopilotPanel.hpp"
#include "application/gui/panels/BaselineAutopilotPanel.hpp"
#include "application/gui/Window.hpp"
#include "application/sim/gnc/TrimTypes.hpp"

namespace sim {
class Aircraft;
} // namespace sim

namespace gui {
class GNCWindow final : public gui::Window {
public:
  GNCWindow();

  AutopilotViewState &GetAutopilotViewState() { return autopilotViewState_; }
  const AutopilotViewState &GetAutopilotViewState() const {
    return autopilotViewState_;
  }

protected:
  void OnRender(gui::GUI &gui) override;

private:
  enum class PendingTrimCommand {
    None,
    RunInitialCondition,
    CurrentState,
  };

  // Trim command handling
  void RequestTrim(PendingTrimCommand command);
  void ExecutePendingTrim(gui::GUI &gui);

  // Trim state
  gnc::TrimRequest trimRequest_;
  bool trimResultOpen_ = true;
  bool trimResidualOpen_ = true;
  bool trimInProgress_ = false;
  PendingTrimCommand pendingTrimCommand_ = PendingTrimCommand::None;

  // Autopilot UI state
  AutopilotViewState autopilotViewState_;
  AutopilotPanelState primaryAutopilotPanelState_;
  BaselineAutopilotPanelState baselineAutopilotPanelState_;
};
} // namespace gui
