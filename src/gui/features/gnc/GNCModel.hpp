#pragma once

#include "gui/panels/AutopilotPanel.hpp"
#include "gui/panels/BaselineAutopilotPanel.hpp"
#include "sim/gnc/TrimTypes.hpp"

namespace gui {
struct GNCModel {
  // Feature-owned editing state
  gnc::TrimRequest trimRequest;
  AutopilotPanelState primaryAutopilot;
  BaselineAutopilotPanelState baselineAutopilot;

  // View and synchronization state
  bool trimResultOpen = true;
  bool trimResidualOpen = true;
  bool trimInProgress = false;
  bool autopilotStateLoaded = false;
};
} // namespace gui
