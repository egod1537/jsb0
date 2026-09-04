#include "gui/windows/SimControlWindow.hpp"

#include "common/Options.hpp"
#include "gui/features/editor/EditorPlatformController.hpp"
#include "gui/layout/Toolbar.hpp"
#include "gui/resources/EditorIconRegistry.hpp"
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
constexpr std::array<int, 3> SimSpeeds = {1, 2, 3};
constexpr std::array<ImGuiKey, 4> SimSpeedShortcutKeys = {
    ImGuiKey_1,
    ImGuiKey_2,
    ImGuiKey_3,
    ImGuiKey_4,
};
constexpr std::array<ImGuiKey, 4> SimSpeedKeypadShortcutKeys = {
    ImGuiKey_Keypad1,
    ImGuiKey_Keypad2,
    ImGuiKey_Keypad3,
    ImGuiKey_Keypad4,
};
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
enum class TransportIcon {
  Play,
  Stop,
  Pause,
  Step,
  Reset,
};

std::string DefaultSimulationTickLabel() {
  return std::to_string(static_cast<int>(opts::simulation::Hz))
         + " Hz fixed tick";
}

ImVec2 Offset(ImVec2 point, float x, float y) {
  return {point.x + x, point.y + y};
}

ImVec2 UiOffset(ImVec2 point, float x, float y) {
  return Offset(point, ui::Ui(x), ui::Ui(y));
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
        ui::Ui(1.0F));
    break;
  case TransportIcon::Pause:
    drawList.AddRectFilled(UiOffset(center, -6.0F, -7.0F),
        UiOffset(center, -2.0F, 7.0F),
        color,
        ui::Ui(1.0F));
    drawList.AddRectFilled(UiOffset(center, 2.0F, -7.0F),
        UiOffset(center, 6.0F, 7.0F),
        color,
        ui::Ui(1.0F));
    break;
  case TransportIcon::Step:
    drawList.AddTriangleFilled(UiOffset(center, -7.0F, -7.0F),
        UiOffset(center, -7.0F, 7.0F),
        UiOffset(center, 4.0F, 0.0F),
        color);
    drawList.AddRectFilled(UiOffset(center, 6.0F, -7.0F),
        UiOffset(center, 9.0F, 7.0F),
        color,
        ui::Ui(1.0F));
    break;
  case TransportIcon::Reset:
    drawList.PathArcTo(center, ui::Ui(7.0F), -0.75F, 4.35F, 24);
    drawList.PathStroke(color, 0, ui::Ui(2.0F));
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
  const float extent = ui::Ui(ToolbarButtonSize);
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
      ui::Ui(ToolbarButtonCornerRadius));
  drawList.AddRect(minimum,
      maximum,
      ImGui::GetColorU32(active ? ImGuiCol_CheckMark : ImGuiCol_Border),
      ui::Ui(ToolbarButtonCornerRadius));
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

float GetStatusBadgeWidth(sim::SimExecutionState state) {
  return ImGui::CalcTextSize(sim::ToString(state)).x + ui::Ui(26.0F);
}

void DrawStatusBadge(sim::SimExecutionState state) {
  const bool isRunning = state == sim::SimExecutionState::Running;
  const char *label = sim::ToString(state);
  const ImVec2 textSize = ImGui::CalcTextSize(label);
  const ImVec2 size{GetStatusBadgeWidth(state),
      ui::Ui(ToolbarButtonSize)};
  const ImVec2 minimum = ImGui::GetCursorScreenPos();
  const ImVec2 maximum = Offset(minimum, size.x, size.y);
  const ImVec4 accent = ui::GetDarkEditorSemanticColor(
      isRunning ? ui::SemanticColor::Success
                : ui::SemanticColor::Warning);

  ImGui::Dummy(size);

  ImDrawList &drawList = *ImGui::GetWindowDrawList();
  drawList.AddRectFilled(minimum,
      maximum,
      ImGui::GetColorU32(ImVec4(accent.x, accent.y, accent.z, 0.13F)),
      ui::Ui(ToolbarButtonCornerRadius));
  drawList.AddRect(minimum,
      maximum,
      ImGui::GetColorU32(ImVec4(accent.x, accent.y, accent.z, 0.55F)),
      ui::Ui(ToolbarButtonCornerRadius));
  drawList.AddCircleFilled(
      Offset(UiOffset(minimum, 10.0F, 0.0F), 0.0F, size.y * 0.5F),
      ui::Ui(3.0F),
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
      ImVec2(ui::Ui(SpeedButtonWidth), ui::Ui(ToolbarButtonSize)));

  if (selected) {
    ImGui::PopStyleColor(2);
  }

  if (ImGui::IsItemHovered() && tooltip != nullptr) {
    ImGui::SetTooltip("%s", tooltip);
  }

  return clicked;
}

float GetCenterGroupWidth() {
  const std::string tickLabel = DefaultSimulationTickLabel();
  return ui::Ui(ToolbarButtonSize * 4.0F + ToolbarButtonSpacing * 5.0F
                      + SpeedButtonWidth * 4.0F + ToolbarSectionSpacing * 3.0F)
         + ImGui::CalcTextSize("Speed").x
         + ImGui::CalcTextSize(tickLabel.c_str()).x;
}

float GetToolbarControlsWidth(sim::SimExecutionState state) {
  return GetStatusBadgeWidth(state)
         + ui::Ui(ToolbarButtonSpacing + ToolbarSectionSpacing)
         + GetCenterGroupWidth();
}

std::string MakeScenarioStatusLabel(
    const sim::ScenarioExecutionStatus &scenario,
    sim::SimExecutionState state) {
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
         + " · " + sim::ToString(state) + " · " + timeLabel;
}

std::string MakeRecordingButtonLabel(
    const telemetry::recording::RecordingStatus &status) {
  if (status.state != telemetry::recording::RecordingState::Recording) {
    return "Record";
  }
  const int elapsedSeconds =
      static_cast<int>(std::max(0.0, std::floor(status.elapsedSimulationSec)));
  char label[64]{};
  std::snprintf(label,
      sizeof(label),
      "Stop Recording %02d:%02d",
      elapsedSeconds / 60,
      elapsedSeconds % 60);
  return label;
}

std::string MakeShortcutLabel(std::size_t index) {
  return index < LayoutShortcutKeys.size() ? "F" + std::to_string(index + 1)
                                           : std::string{};
}
} // namespace

namespace gui {
SimControlWindow::SimControlWindow(
    SimController &simulation, ScenarioController &scenario,
    EditorPlatformController &editorPlatform, EditorIconRegistry &icons)
    : Window("Simulation Control"), simulation_(simulation),
      scenarioPopup_(scenario), editorPlatform_(editorPlatform), icons_(icons) {
}

float SimControlWindow::GetReservedHeight() {
  return ui::Ui(ToolbarHeight);
}

void SimControlWindow::PrepareWindow() {
  const ImGuiViewport *viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->WorkPos, ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, GetReservedHeight()),
      ImGuiCond_Always);
  ImGui::SetNextWindowViewport(viewport->ID);
}

ImGuiWindowFlags SimControlWindow::GetWindowFlags() const {
  return ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize
         | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse
         | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings
         | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoBringToFrontOnFocus
         | ImGuiWindowFlags_NoNavFocus;
}

void SimControlWindow::OnRender(
    const sim::SimSnapshot &snapshot) {
  HandleTransportShortcut();
  HandleLayoutShortcuts();

  const SimTransportProps props = simulation_.GetTransportProps();
  const sim::SimExecutionState state = props.executionState;
  const bool isStopped = state == sim::SimExecutionState::Stopped;
  const bool isRunning = state == sim::SimExecutionState::Running;
  const bool isPaused = state == sim::SimExecutionState::Paused;
  const std::optional<sim::ScenarioExecutionStatus> &scenarioStatus =
      props.scenarioStatus;
  const bool scenarioInactive = !scenarioStatus.has_value();
  HandleSimulationSpeedShortcut(scenarioInactive);
  const bool showScenarioStatus = !isStopped && scenarioStatus.has_value();
  const telemetry::recording::RecordingStatus &recordingStatus =
      props.recordingStatus;

  const float buttonSpacing = ui::Ui(ToolbarButtonSpacing);
  const float sectionSpacing = ui::Ui(ToolbarSectionSpacing);
  Toolbar toolbar;
  const float scenarioButtonWidth = ImGui::CalcTextSize("New Scenario...").x
                                    + ImGui::GetStyle().FramePadding.x * 2.0F;
  toolbar.Left(scenarioButtonWidth, [this, scenarioButtonWidth] {
    if (ImGui::Button("New Scenario...",
            ImVec2(scenarioButtonWidth, ui::Ui(ToolbarButtonSize)))) {
      scenarioPopup_.RequestOpen();
    }
  });
  toolbar.Center(GetToolbarControlsWidth(state), [&] {
    DrawStatusBadge(state);
    ImGui::SameLine(0.0F, buttonSpacing);
    if (DrawTransportButton("ResetSimulation",
            TransportIcon::Reset,
            scenarioInactive && !isStopped,
            false,
            "Reset simulation")) {
      simulation_.OnEvent(SimResetRequested{});
    }

    ImGui::SameLine(0.0F, sectionSpacing);
    if (DrawTransportButton("PlayStop",
            isStopped ? TransportIcon::Play : TransportIcon::Stop,
            scenarioInactive || !isStopped,
            isRunning,
            isStopped ? "Play simulation (Space)"
                      : "Stop simulation (Space)")) {
      simulation_.OnEvent(SimPlaybackToggled{});
    }

    ImGui::SameLine(0.0F, buttonSpacing);
    if (DrawTransportButton("PauseResume",
            isPaused ? TransportIcon::Play : TransportIcon::Pause,
            !isStopped,
            isPaused,
            isPaused ? "Resume simulation" : "Pause simulation")) {
      if (isPaused) {
        simulation_.OnEvent(SimResumeRequested{});
      } else {
        simulation_.OnEvent(SimPauseRequested{});
      }
    }

    ImGui::SameLine(0.0F, buttonSpacing);
    if (DrawTransportButton("StepSimulation",
            TransportIcon::Step,
            isPaused,
            false,
            "Advance exactly one simulation tick")) {
      simulation_.OnEvent(SimStepRequested{});
    }

    ImGui::SameLine(0.0F, sectionSpacing);
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("Speed");

    const double automaticHz = props.automaticHz;
    const bool maximumSpeed = props.maximumSpeed;
    ImGui::BeginDisabled(!scenarioInactive);
    for (const int speed : SimSpeeds) {
      ImGui::SameLine(0.0F, buttonSpacing);
      const double speedHz = opts::simulation::Hz * speed;
      const std::string label = std::to_string(speed) + "x";
      const std::string tooltip = "Run at " + std::to_string(speed)
                                  + "x speed (" + std::to_string(speed) + ")";
      if (DrawSpeedButton(label.c_str(),
              !maximumSpeed && std::abs(automaticHz - speedHz) < 0.5,
              tooltip.c_str())) {
        simulation_.OnEvent(SimRateChanged{speedHz});
      }
    }

    ImGui::SameLine(0.0F, buttonSpacing);
    if (DrawSpeedButton("Max",
            maximumSpeed,
            "Run as fast as the CPU allows (4)")) {
      simulation_.OnEvent(MaximumSimulationSpeedChanged{true});
    }
    ImGui::EndDisabled();

    ImGui::SameLine(0.0F, sectionSpacing);
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("%s", DefaultSimulationTickLabel().c_str());
  });
  const std::string scenarioLabel =
      showScenarioStatus ? MakeScenarioStatusLabel(*scenarioStatus, state)
                         : std::string{};
  const std::string layoutLabel = GetLayoutButtonLabel();
  const std::string recordingLabel = MakeRecordingButtonLabel(recordingStatus);
  const float layoutButtonWidth =
      ImGui::CalcTextSize(layoutLabel.c_str()).x + ui::Ui(36.0F);
  const float recordingButtonWidth =
      ImGui::CalcTextSize(recordingLabel.c_str()).x
      + ImGui::GetStyle().FramePadding.x * 2.0F;
  const float folderButtonWidth =
      ImGui::CalcTextSize("Folder").x + ImGui::GetStyle().FramePadding.x * 2.0F;
  const float rightWidth =
      layoutButtonWidth + recordingButtonWidth + folderButtonWidth
      + sectionSpacing * 2.0F
      + (scenarioLabel.empty()
              ? 0.0F
              : ImGui::CalcTextSize(scenarioLabel.c_str()).x + sectionSpacing);
  toolbar.Right(rightWidth,
      [this,
          scenarioLabel,
          recordingStatus,
          recordingLabel,
          recordingButtonWidth,
          folderButtonWidth,
          layoutButtonWidth,
          sectionSpacing] {
        if (!scenarioLabel.empty()) {
          ImGui::AlignTextToFramePadding();
          ImGui::TextDisabled("%s", scenarioLabel.c_str());
          ImGui::SameLine(0.0F, sectionSpacing);
        }
        if (ImGui::Button(recordingLabel.c_str(),
                ImVec2(recordingButtonWidth,
                    ui::Ui(ToolbarButtonSize)))) {
          simulation_.OnEvent(TelemetryRecordingToggled{});
        }
        if (ImGui::IsItemHovered()) {
          if (recordingStatus.state
                  == telemetry::recording::RecordingState::Error
              && !recordingStatus.errorMessage.empty()) {
            ImGui::SetTooltip("Recording error: %s",
                recordingStatus.errorMessage.c_str());
          } else if (!recordingStatus.outputPath.empty()) {
            ImGui::SetTooltip("%s",
                recordingStatus.outputPath.string().c_str());
          } else {
            ImGui::SetTooltip("Record simulation telemetry to MCAP");
          }
        }
        ImGui::SameLine(0.0F, sectionSpacing);
        if (ImGui::Button("Folder",
                ImVec2(folderButtonWidth, ui::Ui(ToolbarButtonSize)))) {
          simulation_.OnEvent(OpenTelemetryFolderRequested{});
        }
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip("Open recordings folder");
        }
        ImGui::SameLine(0.0F, sectionSpacing);
        DrawLayoutDropdown(layoutButtonWidth);
      });
  toolbar.Render();
  DrawLayoutDialogs();
  scenarioPopup_.Draw(snapshot);
}

void SimControlWindow::HandleTransportShortcut() {
  const ImGuiIO &io = ImGui::GetIO();
  if (io.WantTextInput || io.KeyCtrl || io.KeyShift || io.KeyAlt || io.KeySuper
      || ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId)) {
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Space, false)) {
    simulation_.OnEvent(SimPlaybackToggled{});
  }
}

void SimControlWindow::HandleSimulationSpeedShortcut(bool enabled) {
  const ImGuiIO &io = ImGui::GetIO();
  if (!enabled || io.WantTextInput || io.KeyCtrl || io.KeyShift || io.KeyAlt
      || io.KeySuper || ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId)) {
    return;
  }

  for (std::size_t index = 0; index < SimSpeedShortcutKeys.size();
      ++index) {
    const bool pressed =
        ImGui::IsKeyPressed(SimSpeedShortcutKeys[index], false)
        || ImGui::IsKeyPressed(SimSpeedKeypadShortcutKeys[index], false);
    if (!pressed) {
      continue;
    }
    if (index < SimSpeeds.size()) {
      simulation_.OnEvent(SimRateChanged{
          opts::simulation::Hz * SimSpeeds[index]});
    } else {
      simulation_.OnEvent(MaximumSimulationSpeedChanged{true});
    }
    break;
  }
}

void SimControlWindow::HandleLayoutShortcuts() {
  const ImGuiIO &io = ImGui::GetIO();
  if (io.WantTextInput) {
    return;
  }
  const auto &presets = editorPlatform_.GetPresets();
  const std::size_t shortcutCount =
      std::min(presets.size(), LayoutShortcutKeys.size());
  for (std::size_t index = 0; index < shortcutCount; ++index) {
    if (!ImGui::IsKeyPressed(LayoutShortcutKeys[index], false)) {
      continue;
    }
    const EditorLayoutOperationResult result =
        editorPlatform_.ApplyLayout(presets[index].id);
    if (!result.succeeded) {
      SetLayoutFeedback("Failed to apply layout: " + result.error, true);
    } else {
      SetLayoutFeedback("Applied layout: " + presets[index].name);
    }
    break;
  }
}

void SimControlWindow::DrawLayoutDropdown(float width) {
  const std::string label = GetLayoutButtonLabel();
  const bool clicked = ImGui::Button("##LayoutPresetButton",
      ImVec2(width, ui::Ui(ToolbarButtonSize)));
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
      icons_.Get(editor_icon_aliases::LayoutDropdown);
  const float iconHeight = ui::Ui(10.0F);
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

  const auto &presets = editorPlatform_.GetPresets();
  for (std::size_t index = 0; index < presets.size(); ++index) {
    const EditorLayoutPreset &preset = presets[index];
    const std::string shortcut = MakeShortcutLabel(index);
    const bool selected = editorPlatform_.GetActivePresetId().has_value()
                          && *editorPlatform_.GetActivePresetId() == preset.id;
    if (ImGui::MenuItem(preset.name.c_str(),
            shortcut.empty() ? nullptr : shortcut.c_str(),
            selected)) {
      const EditorLayoutOperationResult result =
          editorPlatform_.ApplyLayout(preset.id);
      if (result.succeeded) {
        SetLayoutFeedback("Applied layout: " + preset.name);
      } else {
        SetLayoutFeedback("Failed to apply layout: " + result.error, true);
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
    ImportLayout();
  }
  if (ImGui::MenuItem("Manage Layouts...")) {
    openManageLayoutsDialog_ = true;
  }
  if (ImGui::MenuItem("Reset to Default Layout")) {
    editorPlatform_.ResetLayout();
    SetLayoutFeedback("Reset to default layout");
  }
  if (!layoutFeedback_.empty()) {
    ImGui::Separator();
    const ImVec4 color = layoutFeedbackIsError_
                             ? ui::GetDarkEditorSemanticColor(
                                   ui::SemanticColor::Error)
                             : ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled);
    ImGui::TextColored(color, "%s", layoutFeedback_.c_str());
  }
  ImGui::EndPopup();
}

void SimControlWindow::DrawLayoutDialogs() {
  if (openSaveLayoutDialog_) {
    ImGui::OpenPopup("Save Current Layout");
    openSaveLayoutDialog_ = false;
  }
  if (openManageLayoutsDialog_) {
    manageLayoutsVisible_ = true;
    ImGui::OpenPopup("Manage Layouts");
    openManageLayoutsDialog_ = false;
  }
  DrawSaveLayoutDialog();
  DrawManageLayoutsDialog();
}

void SimControlWindow::DrawSaveLayoutDialog() {
  if (!ImGui::BeginPopupModal("Save Current Layout",
          nullptr,
          ImGuiWindowFlags_AlwaysAutoResize)) {
    return;
  }
  ImGui::TextUnformatted("Name");
  ImGui::SetNextItemWidth(ui::Ui(320.0F));
  const bool submitted = ImGui::InputText("##LayoutName",
      layoutNameInput_.data(),
      layoutNameInput_.size(),
      ImGuiInputTextFlags_EnterReturnsTrue);
  const bool hasName = layoutNameInput_[0] != '\0';
  ImGui::BeginDisabled(!hasName);
  if (ImGui::Button("Save") || submitted) {
    const EditorLayoutOperationResult result =
        editorPlatform_.SaveLayout(layoutNameInput_.data());
    if (result.succeeded) {
      selectedLayoutId_ = result.presetId;
      SetLayoutFeedback("Saved layout: " + result.presetName);
      ImGui::CloseCurrentPopup();
    } else {
      SetLayoutFeedback("Failed to save layout: " + result.error, true);
    }
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Cancel")) {
    ImGui::CloseCurrentPopup();
  }
  if (!layoutFeedback_.empty() && layoutFeedbackIsError_) {
    ImGui::TextColored(
        ui::GetDarkEditorSemanticColor(ui::SemanticColor::Error),
        "%s",
        layoutFeedback_.c_str());
  }
  ImGui::EndPopup();
}

void SimControlWindow::DrawManageLayoutsDialog() {
  ImGui::SetNextWindowSize(ImVec2(ui::Ui(620.0F), ui::Ui(430.0F)),
      ImGuiCond_Appearing);
  if (!ImGui::BeginPopupModal("Manage Layouts", &manageLayoutsVisible_)) {
    if (!manageLayoutsVisible_) {
      renameLayout_ = false;
    }
    return;
  }

  const auto &presets = editorPlatform_.GetPresets();
  std::optional<std::pair<LayoutPresetId, std::size_t>> pendingMove;
  if (ImGui::BeginTable("LayoutPresetTable",
          2,
          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
              | ImGuiTableFlags_ScrollY,
          ImVec2(0.0F, ui::Ui(260.0F)))) {
    ImGui::TableSetupColumn("Shortcut",
        ImGuiTableColumnFlags_WidthFixed,
        ui::Ui(70.0F));
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
  if (pendingMove.has_value()) {
    const EditorLayoutOperationResult result =
        editorPlatform_.MoveLayout(pendingMove->first, pendingMove->second);
    if (!result.succeeded) {
      SetLayoutFeedback("Failed to reorder layouts: " + result.error, true);
    }
  }

  const EditorLayoutPreset *selected =
      editorPlatform_.FindPreset(selectedLayoutId_);
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
    const EditorLayoutOperationResult result =
        editorPlatform_.UpdateLayout(selected->id);
    if (result.succeeded) {
      SetLayoutFeedback("Updated layout: " + result.presetName);
    } else {
      SetLayoutFeedback("Failed to update layout: " + result.error, true);
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Export") && selected != nullptr) {
    ExportLayout(selected->id);
  }
  ImGui::SameLine();
  if (ImGui::Button("Delete") && selected != nullptr) {
    const EditorLayoutOperationResult result =
        editorPlatform_.DeleteLayout(selected->id);
    if (result.succeeded) {
      selectedLayoutId_.clear();
      renameLayout_ = false;
      SetLayoutFeedback("Deleted layout: " + result.presetName);
    } else {
      SetLayoutFeedback("Failed to delete layout: " + result.error, true);
    }
  }
  ImGui::EndDisabled();

  if (renameLayout_ && selected != nullptr) {
    ImGui::Spacing();
    ImGui::SetNextItemWidth(ui::Ui(320.0F));
    ImGui::InputText("##RenameLayout",
        layoutNameInput_.data(),
        layoutNameInput_.size());
    ImGui::SameLine();
    if (ImGui::Button("Apply Rename")) {
      const EditorLayoutOperationResult result =
          editorPlatform_.RenameLayout(selected->id, layoutNameInput_.data());
      if (result.succeeded) {
        SetLayoutFeedback("Renamed layout");
        renameLayout_ = false;
      } else {
        SetLayoutFeedback("Failed to rename layout: " + result.error, true);
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel Rename")) {
      renameLayout_ = false;
    }
  }

  ImGui::Separator();
  if (ImGui::Button("Import Layout...")) {
    ImportLayout();
  }
  if (!layoutFeedback_.empty()) {
    const ImVec4 color = layoutFeedbackIsError_
                             ? ui::GetDarkEditorSemanticColor(
                                   ui::SemanticColor::Error)
                             : ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled);
    ImGui::TextColored(color, "%s", layoutFeedback_.c_str());
  }
  ImGui::EndPopup();
  if (!manageLayoutsVisible_) {
    renameLayout_ = false;
  }
}

void SimControlWindow::ImportLayout() {
  const EditorLayoutOperationResult result = editorPlatform_.ImportLayout();
  if (result.canceled) {
    return;
  }
  if (!result.succeeded) {
    SetLayoutFeedback("Failed to import layout: " + result.error, true);
    return;
  }
  selectedLayoutId_ = result.presetId;
  SetLayoutFeedback("Import successful: " + result.presetName);
}

void SimControlWindow::ExportLayout(const LayoutPresetId &id) {
  const EditorLayoutOperationResult result = editorPlatform_.ExportLayout(id);
  if (result.canceled) {
    return;
  }
  if (result.succeeded) {
    SetLayoutFeedback("Exported layout: " + result.presetName);
  } else {
    SetLayoutFeedback("Failed to export layout: " + result.error, true);
  }
}

void SimControlWindow::SetLayoutFeedback(std::string message,
    bool isError) {
  layoutFeedback_ = std::move(message);
  layoutFeedbackIsError_ = isError;
}

std::string SimControlWindow::GetLayoutButtonLabel() const {
  if (editorPlatform_.GetActivePresetId().has_value()) {
    const auto &presets = editorPlatform_.GetPresets();
    for (std::size_t index = 0; index < presets.size(); ++index) {
      if (presets[index].id == *editorPlatform_.GetActivePresetId()) {
        const std::string shortcut = MakeShortcutLabel(index);
        return (shortcut.empty() ? std::string{} : shortcut + " · ")
               + presets[index].name;
      }
    }
  }
  return "Layout";
}
} // namespace gui
