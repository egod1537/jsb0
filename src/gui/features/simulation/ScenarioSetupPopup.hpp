#pragma once

#include "sim/runtime/SimContracts.hpp"

namespace gui {
class ScenarioController;

class ScenarioSetupPopup {
public:
  explicit ScenarioSetupPopup(ScenarioController &controller);

  // Popup lifecycle
  void RequestOpen();
  void Cancel();
  bool IsOpenRequested() const { return openRequested_; }
  void Draw(const sim::SimSnapshot &snapshot);

private:
  // Rendering
  void DrawSelection();
  void DrawSummary();
  void DrawActions(const sim::SimSnapshot &snapshot);

  // Dependencies
  ScenarioController &controller_;

  // Modal state
  bool openRequested_ = false;
  bool closeRequested_ = false;
};
} // namespace gui
