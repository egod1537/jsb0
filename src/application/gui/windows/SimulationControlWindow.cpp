#include "application/gui/windows/SimulationControlWindow.hpp"

#include "application/gui/GUI.hpp"
#include "application/gui/layout/Toolbar.hpp"
#include "application/gui/platform/FileDialogService.hpp"
#include "application/sim/Simulation.hpp"
#include "flightui/core/Theme.hpp"
#include "flightui/core/UIScale.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>

namespace {
constexpr float ToolbarHeight = 46.0F;
constexpr float ToolbarButtonSize = 30.0F;
constexpr float SpeedButtonWidth = 32.0F;
constexpr float ToolbarButtonSpacing = 5.0F;
constexpr float ToolbarSectionSpacing = 10.0F;
constexpr float ToolbarButtonCornerRadius = 4.0F;
constexpr std::array<int, 3> SimulationSpeeds = {1, 2, 3};
constexpr std::array<ImGuiKey, 12> LayoutShortcutKeys = {
    ImGuiKey_F1,
    ImGuiKey_F2,
    ImGuiKey_F3,
    ImGuiKey_F4,
    ImGuiKey_F5,
    ImGuiKey_F6,
    ImGuiKey_F7,
    ImGuiKey_F8,
    ImGuiKey_F9,
    ImGuiKey_F10,
    ImGuiKey_F11,
    ImGuiKey_F12,
};
const gui::FileDialogFilter LayoutFileFilter{
    .displayName = "JSB Editor Layout (*.layout.json)",
    .pattern = "*.layout.json",
};

enum class TransportIcon {
  Play,
  Stop,
  Pause,
  Step,
  Reset,
};

ImVec2 Offset(ImVec2 point, float x, float y) {
  return {point.x + x, point.y + y};
}

ImVec2 UiOffset(ImVec2 point, float x, float y) {
  return Offset(point, FlightUI::Ui(x), FlightUI::Ui(y));
}

void DrawTransportIcon(ImDrawList &drawList, TransportIcon icon, ImVec2 center,
    ImU32 color) {
  switch (icon) {
  case TransportIcon::Play:
    drawList.AddTriangleFilled(UiOffset(center, -5.0F, -7.0F),
        UiOffset(center, -5.0F, 7.0F),
        UiOffset(center, 7.0F, 0.0F),
        color);
    break;
  case TransportIcon::Stop:
    drawList.AddRectFilled(UiOffset(center, -6.0F, -6.0F),
        UiOffset(center, 6.0F, 6.0F),
        color,
        FlightUI::Ui(1.0F));
    break;
  case TransportIcon::Pause:
    drawList.AddRectFilled(UiOffset(center, -6.0F, -7.0F),
        UiOffset(center, -2.0F, 7.0F),
        color,
        FlightUI::Ui(1.0F));
    drawList.AddRectFilled(UiOffset(center, 2.0F, -7.0F),
        UiOffset(center, 6.0F, 7.0F),
        color,
        FlightUI::Ui(1.0F));
    break;
  case TransportIcon::Step:
    drawList.AddTriangleFilled(UiOffset(center, -7.0F, -7.0F),
        UiOffset(center, -7.0F, 7.0F),
        UiOffset(center, 4.0F, 0.0F),
        color);
    drawList.AddRectFilled(UiOffset(center, 6.0F, -7.0F),
        UiOffset(center, 9.0F, 7.0F),
        color,
        FlightUI::Ui(1.0F));
    break;
  case TransportIcon::Reset:
    drawList.PathArcTo(center, FlightUI::Ui(7.0F), -0.75F, 4.35F, 24);
    drawList.PathStroke(color, 0, FlightUI::Ui(2.0F));
    drawList.AddTriangleFilled(UiOffset(center, -6.5F, -5.5F),
        UiOffset(center, -8.5F, -0.5F),
        UiOffset(center, -3.0F, -2.0F),
        color);
    break;
  }
}

bool DrawTransportButton(const char *id, TransportIcon icon, bool enabled,
    bool active, const char *tooltip) {
  ImGui::PushID(id);
  ImGui::BeginDisabled(!enabled);

  const ImVec2 minimum = ImGui::GetCursorScreenPos();
  const float extent = FlightUI::Ui(ToolbarButtonSize);
  const ImVec2 size{extent, extent};
  const bool clicked = ImGui::InvisibleButton("##Button", size);
  const bool hovered =
      ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
  const bool held = ImGui::IsItemActive();

  ImGui::EndDisabled();

  ImGuiCol backgroundColor = active ? ImGuiCol_ButtonActive : ImGuiCol_Button;
  if (held) {
    backgroundColor = ImGuiCol_ButtonActive;
  } else if (hovered && enabled) {
    backgroundColor = ImGuiCol_ButtonHovered;
  }

  ImDrawList &drawList = *ImGui::GetWindowDrawList();
  const ImVec2 maximum = Offset(minimum, size.x, size.y);
  drawList.AddRectFilled(minimum,
      maximum,
      ImGui::GetColorU32(backgroundColor),
      FlightUI::Ui(ToolbarButtonCornerRadius));
  drawList.AddRect(minimum,
      maximum,
      ImGui::GetColorU32(active ? ImGuiCol_CheckMark : ImGuiCol_Border),
      FlightUI::Ui(ToolbarButtonCornerRadius));
  DrawTransportIcon(drawList,
      icon,
      Offset(minimum, size.x * 0.5F, size.y * 0.5F),
      ImGui::GetColorU32(enabled ? ImGuiCol_Text : ImGuiCol_TextDisabled));

  if (hovered && tooltip != nullptr) {
    ImGui::SetTooltip("%s", tooltip);
  }

  ImGui::PopID();
  return clicked && enabled;
}

float GetStatusBadgeWidth(application::SimulationExecutionState state) {
  return ImGui::CalcTextSize(application::ToString(state)).x
         + FlightUI::Ui(26.0F);
}

void DrawStatusBadge(application::SimulationExecutionState state) {
  const bool isRunning =
      state == application::SimulationExecutionState::Running;
  const char *label = application::ToString(state);
  const ImVec2 textSize = ImGui::CalcTextSize(label);
  const ImVec2 size{GetStatusBadgeWidth(state),
      FlightUI::Ui(ToolbarButtonSize)};
  const ImVec2 minimum = ImGui::GetCursorScreenPos();
  const ImVec2 maximum = Offset(minimum, size.x, size.y);
  const ImVec4 accent = FlightUI::GetDarkEditorSemanticColor(
      isRunning ? FlightUI::SemanticColor::Success
                : FlightUI::SemanticColor::Warning);

  ImGui::Dummy(size);

  ImDrawList &drawList = *ImGui::GetWindowDrawList();
  drawList.AddRectFilled(minimum,
      maximum,
      ImGui::GetColorU32(ImVec4(accent.x, accent.y, accent.z, 0.13F)),
      FlightUI::Ui(ToolbarButtonCornerRadius));
  drawList.AddRect(minimum,
      maximum,
      ImGui::GetColorU32(ImVec4(accent.x, accent.y, accent.z, 0.55F)),
      FlightUI::Ui(ToolbarButtonCornerRadius));
  drawList.AddCircleFilled(
      Offset(UiOffset(minimum, 10.0F, 0.0F), 0.0F, size.y * 0.5F),
      FlightUI::Ui(3.0F),
      ImGui::GetColorU32(accent));
  drawList.AddText(Offset(UiOffset(minimum, 18.0F, 0.0F),
                       0.0F,
                       (size.y - textSize.y) * 0.5F),
      ImGui::GetColorU32(ImGuiCol_Text),
      label);
}

bool DrawSpeedButton(const char *label, bool selected, const char *tooltip) {
  if (selected) {
    ImGui::PushStyleColor(ImGuiCol_Button,
        ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    ImGui::PushStyleColor(ImGuiCol_Border,
        ImGui::GetStyleColorVec4(ImGuiCol_CheckMark));
  }

  const bool clicked = ImGui::Button(label,
      ImVec2(FlightUI::Ui(SpeedButtonWidth), FlightUI::Ui(ToolbarButtonSize)));

  if (selected) {
    ImGui::PopStyleColor(2);
  }

  if (ImGui::IsItemHovered() && tooltip != nullptr) {
    ImGui::SetTooltip("%s", tooltip);
  }

  return clicked;
}

float GetCenterGroupWidth() {
  return FlightUI::Ui(ToolbarButtonSize * 4.0F + ToolbarButtonSpacing * 5.0F
                      + SpeedButtonWidth * 4.0F + ToolbarSectionSpacing * 3.0F)
         + ImGui::CalcTextSize("Speed").x
         + ImGui::CalcTextSize("30 Hz fixed tick").x;
}

float GetToolbarControlsWidth(application::SimulationExecutionState state) {
  return GetStatusBadgeWidth(state)
         + FlightUI::Ui(ToolbarButtonSpacing + ToolbarSectionSpacing)
         + GetCenterGroupWidth();
}

std::string MakeScenarioStatusLabel(
    const application::ScenarioExecutionStatus &scenario,
    application::SimulationExecutionState state) {
  const double elapsedSec = std::isfinite(scenario.elapsedSec)
                                ? std::max(scenario.elapsedSec, 0.0)
                                : 0.0;
  const double durationSec = std::isfinite(scenario.durationSec)
                                 ? std::max(scenario.durationSec, 0.0)
                                 : 0.0;
  char timeLabel[64]{};
  std::snprintf(timeLabel,
      sizeof(timeLabel),
      "%.1f / %.1f s",
      elapsedSec,
      durationSec);
  return "Scenario · "
         + (scenario.name.empty() ? std::string("Unnamed") : scenario.name)
         + " · " + application::ToString(state) + " · " + timeLabel;
}

std::string MakeShortcutLabel(std::size_t index) {
  return index < LayoutShortcutKeys.size() ? "F" + std::to_string(index + 1)
                                           : std::string{};
}
} // namespace

namespace gui {
SimulationControlWindow::SimulationControlWindow()
    : Window("Simulation Control") {}

float SimulationControlWindow::GetReservedHeight() {
  return FlightUI::Ui(ToolbarHeight);
}

void SimulationControlWindow::PrepareWindow() {
  const ImGuiViewport *viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->WorkPos, ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, GetReservedHeight()),
      ImGuiCond_Always);
  ImGui::SetNextWindowViewport(viewport->ID);
}

ImGuiWindowFlags SimulationControlWindow::GetWindowFlags() const {
  return ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize
         | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse
         | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings
         | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoBringToFrontOnFocus
         | ImGuiWindowFlags_NoNavFocus;
}

void SimulationControlWindow::OnRender(GUI &gui) {
  HandleLayoutShortcuts(gui);

  auto &executionControl = gui.GetSimulationExecutionControl();
  const application::SimulationExecutionState state =
      executionControl.GetSimulationExecutionState();
  const bool isStopped =
      state == application::SimulationExecutionState::Stopped;
  const bool isRunning =
      state == application::SimulationExecutionState::Running;
  const bool isPaused = state == application::SimulationExecutionState::Paused;
  const std::optional<application::ScenarioExecutionStatus> scenarioStatus =
      executionControl.GetScenarioExecutionStatus();
  const bool scenarioInactive = !scenarioStatus.has_value();
  const bool showScenarioStatus = !isStopped && scenarioStatus.has_value();

  const float buttonSpacing = FlightUI::Ui(ToolbarButtonSpacing);
  const float sectionSpacing = FlightUI::Ui(ToolbarSectionSpacing);
  Toolbar toolbar;
  toolbar.Center(GetToolbarControlsWidth(state), [&] {
    DrawStatusBadge(state);
    ImGui::SameLine(0.0F, buttonSpacing);
    if (DrawTransportButton("ResetSimulation",
            TransportIcon::Reset,
            scenarioInactive && !isStopped,
            false,
            "Reset simulation")) {
      const bool resumeAfterReset = isRunning;
      executionControl.PauseSimulation();
      if (executionControl.ResetSimulation() && resumeAfterReset) {
        executionControl.ResumeSimulation();
      }
    }

    ImGui::SameLine(0.0F, sectionSpacing);
    if (DrawTransportButton("PlayStop",
            isStopped ? TransportIcon::Play : TransportIcon::Stop,
            scenarioInactive || !isStopped,
            isRunning,
            isStopped ? "Play simulation" : "Stop simulation")) {
      if (isStopped) {
        executionControl.StartSimulation();
      } else {
        executionControl.StopSimulation();
      }
    }

    ImGui::SameLine(0.0F, buttonSpacing);
    if (DrawTransportButton("PauseResume",
            isPaused ? TransportIcon::Play : TransportIcon::Pause,
            !isStopped,
            isPaused,
            isPaused ? "Resume simulation" : "Pause simulation")) {
      if (isPaused) {
        executionControl.ResumeSimulation();
      } else {
        executionControl.PauseSimulation();
      }
    }

    ImGui::SameLine(0.0F, buttonSpacing);
    if (DrawTransportButton("StepSimulation",
            TransportIcon::Step,
            isPaused,
            false,
            "Advance exactly one simulation tick")) {
      executionControl.RequestSimulationTick();
    }

    ImGui::SameLine(0.0F, sectionSpacing);
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("Speed");

    const double automaticHz = executionControl.GetAutomaticSimulationHz();
    const bool maximumSpeed =
        executionControl.IsMaximumSimulationSpeedEnabled();
    ImGui::BeginDisabled(!scenarioInactive);
    for (const int speed : SimulationSpeeds) {
      ImGui::SameLine(0.0F, buttonSpacing);
      const double speedHz = sim::DefaultSimulationHz * speed;
      const std::string label = std::to_string(speed) + "x";
      const std::string tooltip = "Run at " + std::to_string(speed) + "x speed";
      if (DrawSpeedButton(label.c_str(),
              !maximumSpeed && std::abs(automaticHz - speedHz) < 0.5,
              tooltip.c_str())) {
        executionControl.SetAutomaticSimulationHz(speedHz);
      }
    }

    ImGui::SameLine(0.0F, buttonSpacing);
    if (DrawSpeedButton("Max", maximumSpeed, "Run as fast as the CPU allows")) {
      executionControl.SetMaximumSimulationSpeedEnabled(true);
    }
    ImGui::EndDisabled();

    ImGui::SameLine(0.0F, sectionSpacing);
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("30 Hz fixed tick");
  });
  const std::string scenarioLabel =
      showScenarioStatus ? MakeScenarioStatusLabel(*scenarioStatus, state)
                         : std::string{};
  const std::string layoutLabel = GetLayoutButtonLabel(gui.GetEditorLayouts());
  const float layoutButtonWidth =
      ImGui::CalcTextSize(layoutLabel.c_str()).x + FlightUI::Ui(36.0F);
  const float rightWidth =
      layoutButtonWidth
      + (scenarioLabel.empty()
              ? 0.0F
              : ImGui::CalcTextSize(scenarioLabel.c_str()).x + sectionSpacing);
  toolbar.Right(rightWidth,
      [this, &gui, scenarioLabel, layoutButtonWidth, sectionSpacing] {
        if (!scenarioLabel.empty()) {
          ImGui::AlignTextToFramePadding();
          ImGui::TextDisabled("%s", scenarioLabel.c_str());
          ImGui::SameLine(0.0F, sectionSpacing);
        }
        DrawLayoutDropdown(gui, layoutButtonWidth);
      });
  toolbar.Render();
  DrawLayoutDialogs(gui);
}

void SimulationControlWindow::HandleLayoutShortcuts(GUI &gui) {
  const ImGuiIO &io = ImGui::GetIO();
  if (io.WantTextInput) {
    return;
  }
  const auto &presets = gui.GetEditorLayouts().GetPresets();
  const std::size_t shortcutCount =
      std::min(presets.size(), LayoutShortcutKeys.size());
  for (std::size_t index = 0; index < shortcutCount; ++index) {
    if (!ImGui::IsKeyPressed(LayoutShortcutKeys[index], false)) {
      continue;
    }
    if (!gui.GetEditorLayouts().ApplyPreset(presets[index].id)) {
      SetLayoutFeedback("Failed to apply layout: "
                            + gui.GetEditorLayouts().GetLastError(),
          true);
    } else {
      SetLayoutFeedback("Applied layout: " + presets[index].name);
    }
    break;
  }
}

void SimulationControlWindow::DrawLayoutDropdown(GUI &gui, float width) {
  const std::string label = GetLayoutButtonLabel(gui.GetEditorLayouts());
  const bool clicked = ImGui::Button("##LayoutPresetButton",
      ImVec2(width, FlightUI::Ui(ToolbarButtonSize)));
  const ImVec2 minimum = ImGui::GetItemRectMin();
  const ImVec2 maximum = ImGui::GetItemRectMax();
  const ImVec2 textSize = ImGui::CalcTextSize(label.c_str());
  const ImGuiStyle &style = ImGui::GetStyle();
  ImGui::GetWindowDrawList()->AddText(
      ImVec2(minimum.x + style.FramePadding.x,
          minimum.y + (maximum.y - minimum.y - textSize.y) * 0.5F),
      ImGui::GetColorU32(ImGuiCol_Text),
      label.c_str());

  const EditorIconHandle dropdownIcon =
      gui.GetEditorIcons().Get(EditorIconAliases::LayoutDropdown);
  const float iconHeight = FlightUI::Ui(10.0F);
  const float iconWidth =
      dropdownIcon.IsValid()
          ? iconHeight * dropdownIcon.size.x / dropdownIcon.size.y
          : iconHeight;
  const ImVec2 iconMinimum{
      maximum.x - style.FramePadding.x - iconWidth,
      minimum.y + (maximum.y - minimum.y - iconHeight) * 0.5F,
  };
  const ImVec2 iconMaximum{
      iconMinimum.x + iconWidth,
      iconMinimum.y + iconHeight,
  };
  if (dropdownIcon.IsValid()) {
    ImGui::GetWindowDrawList()->AddImage(ImTextureRef(dropdownIcon.texture),
        iconMinimum,
        iconMaximum,
        ImVec2(0.0F, 0.0F),
        ImVec2(1.0F, 1.0F),
        ImGui::GetColorU32(ImGuiCol_Text));
  } else {
    ImGui::GetWindowDrawList()->AddTriangleFilled(iconMinimum,
        ImVec2(iconMaximum.x, iconMinimum.y),
        ImVec2((iconMinimum.x + iconMaximum.x) * 0.5F, iconMaximum.y),
        ImGui::GetColorU32(ImGuiCol_Text));
  }

  if (clicked) {
    ImGui::OpenPopup("LayoutPresetDropdown");
  }
  if (!ImGui::BeginPopup("LayoutPresetDropdown")) {
    return;
  }

  EditorLayoutManager &manager = gui.GetEditorLayouts();
  const auto &presets = manager.GetPresets();
  for (std::size_t index = 0; index < presets.size(); ++index) {
    const EditorLayoutPreset &preset = presets[index];
    const std::string shortcut = MakeShortcutLabel(index);
    const bool selected = manager.GetActivePresetId().has_value()
                          && *manager.GetActivePresetId() == preset.id;
    if (ImGui::MenuItem(preset.name.c_str(),
            shortcut.empty() ? nullptr : shortcut.c_str(),
            selected)) {
      if (manager.ApplyPreset(preset.id)) {
        SetLayoutFeedback("Applied layout: " + preset.name);
      } else {
        SetLayoutFeedback("Failed to apply layout: " + manager.GetLastError(),
            true);
      }
    }
  }
  if (!presets.empty()) {
    ImGui::Separator();
  }
  if (ImGui::MenuItem("Save Current Layout...")) {
    layoutNameInput_.fill('\0');
    openSaveLayoutDialog_ = true;
  }
  if (ImGui::MenuItem("Import Layout...")) {
    ImportLayout(gui);
  }
  if (ImGui::MenuItem("Manage Layouts...")) {
    openManageLayoutsDialog_ = true;
  }
  if (ImGui::MenuItem("Reset to Default Layout")) {
    gui.ResetEditorLayoutToDefault();
    SetLayoutFeedback("Reset to default layout");
  }
  if (!layoutFeedback_.empty()) {
    ImGui::Separator();
    const ImVec4 color = layoutFeedbackIsError_
                             ? FlightUI::GetDarkEditorSemanticColor(
                                   FlightUI::SemanticColor::Error)
                             : ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled);
    ImGui::TextColored(color, "%s", layoutFeedback_.c_str());
  }
  ImGui::EndPopup();
}

void SimulationControlWindow::DrawLayoutDialogs(GUI &gui) {
  if (openSaveLayoutDialog_) {
    ImGui::OpenPopup("Save Current Layout");
    openSaveLayoutDialog_ = false;
  }
  if (openManageLayoutsDialog_) {
    manageLayoutsVisible_ = true;
    ImGui::OpenPopup("Manage Layouts");
    openManageLayoutsDialog_ = false;
  }
  DrawSaveLayoutDialog(gui);
  DrawManageLayoutsDialog(gui);
}

void SimulationControlWindow::DrawSaveLayoutDialog(GUI &gui) {
  if (!ImGui::BeginPopupModal("Save Current Layout",
          nullptr,
          ImGuiWindowFlags_AlwaysAutoResize)) {
    return;
  }
  ImGui::TextUnformatted("Name");
  ImGui::SetNextItemWidth(FlightUI::Ui(320.0F));
  const bool submitted = ImGui::InputText("##LayoutName",
      layoutNameInput_.data(),
      layoutNameInput_.size(),
      ImGuiInputTextFlags_EnterReturnsTrue);
  const bool hasName = layoutNameInput_[0] != '\0';
  ImGui::BeginDisabled(!hasName);
  if (ImGui::Button("Save") || submitted) {
    LayoutPresetId id;
    if (gui.GetEditorLayouts().SaveCurrentLayout(layoutNameInput_.data(),
            &id)) {
      selectedLayoutId_ = id;
      SetLayoutFeedback(
          "Saved layout: " + gui.GetEditorLayouts().FindPreset(id)->name);
      ImGui::CloseCurrentPopup();
    } else {
      SetLayoutFeedback("Failed to save layout: "
                            + gui.GetEditorLayouts().GetLastError(),
          true);
    }
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Cancel")) {
    ImGui::CloseCurrentPopup();
  }
  if (!layoutFeedback_.empty() && layoutFeedbackIsError_) {
    ImGui::TextColored(
        FlightUI::GetDarkEditorSemanticColor(FlightUI::SemanticColor::Error),
        "%s",
        layoutFeedback_.c_str());
  }
  ImGui::EndPopup();
}

void SimulationControlWindow::DrawManageLayoutsDialog(GUI &gui) {
  ImGui::SetNextWindowSize(ImVec2(FlightUI::Ui(620.0F), FlightUI::Ui(430.0F)),
      ImGuiCond_Appearing);
  if (!ImGui::BeginPopupModal("Manage Layouts", &manageLayoutsVisible_)) {
    if (!manageLayoutsVisible_) {
      renameLayout_ = false;
    }
    return;
  }

  EditorLayoutManager &manager = gui.GetEditorLayouts();
  const auto &presets = manager.GetPresets();
  std::optional<std::pair<LayoutPresetId, std::size_t>> pendingMove;
  if (ImGui::BeginTable("LayoutPresetTable",
          2,
          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
              | ImGuiTableFlags_ScrollY,
          ImVec2(0.0F, FlightUI::Ui(260.0F)))) {
    ImGui::TableSetupColumn("Shortcut",
        ImGuiTableColumnFlags_WidthFixed,
        FlightUI::Ui(70.0F));
    ImGui::TableSetupColumn("Layout", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableHeadersRow();
    for (std::size_t index = 0; index < presets.size(); ++index) {
      const EditorLayoutPreset &preset = presets[index];
      ImGui::PushID(preset.id.c_str());
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      const std::string shortcut = MakeShortcutLabel(index);
      ImGui::TextDisabled("%s", shortcut.empty() ? "-" : shortcut.c_str());
      ImGui::TableSetColumnIndex(1);
      if (ImGui::Selectable(preset.name.c_str(),
              selectedLayoutId_ == preset.id,
              ImGuiSelectableFlags_SpanAllColumns)) {
        selectedLayoutId_ = preset.id;
        renameLayout_ = false;
      }
      if (ImGui::BeginDragDropSource()) {
        ImGui::SetDragDropPayload("EDITOR_LAYOUT_PRESET",
            preset.id.c_str(),
            preset.id.size() + 1);
        ImGui::TextUnformatted(preset.name.c_str());
        ImGui::EndDragDropSource();
      }
      if (ImGui::BeginDragDropTarget()) {
        if (const ImGuiPayload *payload =
                ImGui::AcceptDragDropPayload("EDITOR_LAYOUT_PRESET")) {
          pendingMove = std::make_pair(
              LayoutPresetId(static_cast<const char *>(payload->Data)),
              index);
        }
        ImGui::EndDragDropTarget();
      }
      ImGui::PopID();
    }
    ImGui::EndTable();
  }
  if (pendingMove.has_value()
      && !manager.MovePreset(pendingMove->first, pendingMove->second)) {
    SetLayoutFeedback("Failed to reorder layouts: " + manager.GetLastError(),
        true);
  }

  const EditorLayoutPreset *selected = manager.FindPreset(selectedLayoutId_);
  ImGui::BeginDisabled(selected == nullptr);
  if (ImGui::Button("Rename") && selected != nullptr) {
    layoutNameInput_.fill('\0');
    std::strncpy(layoutNameInput_.data(),
        selected->name.c_str(),
        layoutNameInput_.size() - 1);
    renameLayout_ = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Update") && selected != nullptr) {
    if (manager.UpdatePreset(selected->id)) {
      SetLayoutFeedback("Updated layout: " + selected->name);
    } else {
      SetLayoutFeedback("Failed to update layout: " + manager.GetLastError(),
          true);
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Export") && selected != nullptr) {
    ExportLayout(gui, selected->id);
  }
  ImGui::SameLine();
  if (ImGui::Button("Delete") && selected != nullptr) {
    const std::string deletedName = selected->name;
    if (manager.DeletePreset(selected->id)) {
      selectedLayoutId_.clear();
      renameLayout_ = false;
      SetLayoutFeedback("Deleted layout: " + deletedName);
    } else {
      SetLayoutFeedback("Failed to delete layout: " + manager.GetLastError(),
          true);
    }
  }
  ImGui::EndDisabled();

  if (renameLayout_ && selected != nullptr) {
    ImGui::Spacing();
    ImGui::SetNextItemWidth(FlightUI::Ui(320.0F));
    ImGui::InputText("##RenameLayout",
        layoutNameInput_.data(),
        layoutNameInput_.size());
    ImGui::SameLine();
    if (ImGui::Button("Apply Rename")) {
      if (manager.RenamePreset(selected->id, layoutNameInput_.data())) {
        SetLayoutFeedback("Renamed layout");
        renameLayout_ = false;
      } else {
        SetLayoutFeedback("Failed to rename layout: " + manager.GetLastError(),
            true);
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel Rename")) {
      renameLayout_ = false;
    }
  }

  ImGui::Separator();
  if (ImGui::Button("Import Layout...")) {
    ImportLayout(gui);
  }
  if (!layoutFeedback_.empty()) {
    const ImVec4 color = layoutFeedbackIsError_
                             ? FlightUI::GetDarkEditorSemanticColor(
                                   FlightUI::SemanticColor::Error)
                             : ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled);
    ImGui::TextColored(color, "%s", layoutFeedback_.c_str());
  }
  ImGui::EndPopup();
  if (!manageLayoutsVisible_) {
    renameLayout_ = false;
  }
}

void SimulationControlWindow::ImportLayout(GUI &gui) {
  const std::optional<std::filesystem::path> source =
      gui.GetFileDialog().OpenFile("Import Layout", LayoutFileFilter);
  if (!source.has_value()) {
    if (!gui.GetFileDialog().GetLastError().empty()) {
      SetLayoutFeedback("Failed to import layout: "
                            + gui.GetFileDialog().GetLastError(),
          true);
    }
    return;
  }

  LayoutPresetId importedId;
  if (!gui.GetEditorLayouts().ImportPreset(*source, &importedId)) {
    SetLayoutFeedback("Failed to import layout: "
                          + gui.GetEditorLayouts().GetLastError(),
        true);
    return;
  }
  selectedLayoutId_ = importedId;
  const EditorLayoutPreset *preset =
      gui.GetEditorLayouts().FindPreset(importedId);
  SetLayoutFeedback(
      "Import successful: "
      + (preset == nullptr ? std::string("Layout") : preset->name));
}

void SimulationControlWindow::ExportLayout(GUI &gui, const LayoutPresetId &id) {
  const EditorLayoutPreset *preset = gui.GetEditorLayouts().FindPreset(id);
  if (preset == nullptr) {
    SetLayoutFeedback("Failed to export layout: preset was not found", true);
    return;
  }
  const std::optional<std::filesystem::path> destination =
      gui.GetFileDialog().SaveFile("Export Layout",
          LayoutFileFilter,
          EditorLayoutManager::MakeSuggestedExportFileName(preset->name));
  if (!destination.has_value()) {
    if (!gui.GetFileDialog().GetLastError().empty()) {
      SetLayoutFeedback("Failed to export layout: "
                            + gui.GetFileDialog().GetLastError(),
          true);
    }
    return;
  }
  if (gui.GetEditorLayouts().ExportPreset(id, *destination)) {
    SetLayoutFeedback("Exported layout: " + preset->name);
  } else {
    SetLayoutFeedback("Failed to export layout: "
                          + gui.GetEditorLayouts().GetLastError(),
        true);
  }
}

void SimulationControlWindow::SetLayoutFeedback(std::string message,
    bool isError) {
  layoutFeedback_ = std::move(message);
  layoutFeedbackIsError_ = isError;
}

std::string SimulationControlWindow::GetLayoutButtonLabel(
    const EditorLayoutManager &manager) const {
  if (manager.GetActivePresetId().has_value()) {
    const auto &presets = manager.GetPresets();
    for (std::size_t index = 0; index < presets.size(); ++index) {
      if (presets[index].id == *manager.GetActivePresetId()) {
        const std::string shortcut = MakeShortcutLabel(index);
        return (shortcut.empty() ? std::string{} : shortcut + " · ")
               + presets[index].name;
      }
    }
  }
  return "Layout";
}
} // namespace gui
