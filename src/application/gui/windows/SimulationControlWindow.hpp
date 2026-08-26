#pragma once

#include "application/gui/Window.hpp"
#include "application/gui/layout/EditorLayoutManager.hpp"

#include <array>
#include <string>

namespace gui {
class SimulationControlWindow final : public Window {
public:
  // Lifetime and layout
  SimulationControlWindow();
  static float GetReservedHeight();

protected:
  // Window configuration and rendering
  void PrepareWindow() override;
  ImGuiWindowFlags GetWindowFlags() const override;
  void OnRender(GUI &gui) override;

private:
  // Layout preset controls
  void HandleLayoutShortcuts(GUI &gui);
  void DrawLayoutDropdown(GUI &gui, float width);
  void DrawLayoutDialogs(GUI &gui);
  void DrawSaveLayoutDialog(GUI &gui);
  void DrawManageLayoutsDialog(GUI &gui);
  void ImportLayout(GUI &gui);
  void ExportLayout(GUI &gui, const LayoutPresetId &id);
  void SetLayoutFeedback(std::string message, bool isError = false);
  std::string GetLayoutButtonLabel(const EditorLayoutManager &manager) const;

  // Layout dialog state
  std::array<char, 257> layoutNameInput_{};
  LayoutPresetId selectedLayoutId_;
  std::string layoutFeedback_;
  bool layoutFeedbackIsError_ = false;
  bool openSaveLayoutDialog_ = false;
  bool openManageLayoutsDialog_ = false;
  bool manageLayoutsVisible_ = false;
  bool renameLayout_ = false;
};
} // namespace gui
