#include "gui/features/simulation/ScenarioSetupPopup.hpp"

#include "common/math/Math.hpp"
#include "flightui/FlightUI.hpp"
#include "gui/features/simulation/ScenarioController.hpp"

#include <imgui.h>

#include <cstdio>
#include <string>
#include <vector>

namespace gui {

namespace {
constexpr const char *PopupId = "New Scenario...";
constexpr float PopupWidth = 560.0F;
constexpr float PopupHeight = 620.0F;
constexpr float LabelWidth = 150.0F;

int VariantIndex(sim::ExecutionVariant variant) {
  return variant == sim::ExecutionVariant::Baseline ? 0 : 1;
}

sim::ExecutionVariant VariantFromIndex(int index) {
  return index == 0 ? sim::ExecutionVariant::Baseline
                    : sim::ExecutionVariant::Primary;
}

const char *TrimModeLabel(gnc::TrimMode mode) {
  switch (mode) {
  case gnc::TrimMode::Longitudinal:
    return "Longitudinal";
  case gnc::TrimMode::Full:
    return "Full";
  case gnc::TrimMode::Ground:
    return "Ground";
  }
  return "Unknown";
}

ui::PropertyGridBuilder MakeGrid(const char *id) {
  return ui::PropertyGrid(id)
      .LabelWidth(LabelWidth)
      .ColumnSpacing(6.0F)
      .RowPadding(2.0F)
      .AlternatingRows();
}
} // namespace

ScenarioSetupPopup::ScenarioSetupPopup(ScenarioController &controller)
    : controller_(controller) {}

void ScenarioSetupPopup::RequestOpen() {
  controller_.RefreshAvailableScenarios();
  openRequested_ = true;
  closeRequested_ = false;
}

void ScenarioSetupPopup::Cancel() {
  openRequested_ = false;
  closeRequested_ = true;
}

void ScenarioSetupPopup::Draw(const sim::SimSnapshot &snapshot) {
  if (openRequested_) {
    ImGui::OpenPopup(PopupId);
    openRequested_ = false;
  }

  ImGui::SetNextWindowSize(ImVec2(ui::Ui(PopupWidth), ui::Ui(PopupHeight)),
      ImGuiCond_Appearing);
  if (!ImGui::BeginPopupModal(PopupId,
          nullptr,
          ImGuiWindowFlags_NoSavedSettings)) {
    return;
  }

  DrawSelection();
  ImGui::Spacing();
  DrawSummary();
  DrawActions(snapshot);

  if (closeRequested_) {
    ImGui::CloseCurrentPopup();
    closeRequested_ = false;
  }
  ImGui::EndPopup();
}

void ScenarioSetupPopup::DrawSelection() {
  const ScenarioFileModel &model = controller_.GetModel();
  std::vector<std::string> labels;
  labels.reserve(model.availableScenarioFiles.size());
  int selectedIndex = -1;
  for (std::size_t index = 0; index < model.availableScenarioFiles.size();
      ++index) {
    const std::filesystem::path &path = model.availableScenarioFiles[index];
    labels.push_back(path.filename().string());
    if (!model.currentFilePath.empty()
        && path.lexically_normal()
               == model.currentFilePath.lexically_normal()) {
      selectedIndex = static_cast<int>(index);
    }
  }

  ui::PropertyGridBuilder fields = MakeGrid("ScenarioSetupSelection");
  if (labels.empty()) {
    fields.Add("Scenario Selection",
        ui::TextDisabled("No .yaml scenarios found"));
  } else {
    const std::vector<std::filesystem::path> paths =
        model.availableScenarioFiles;
    fields.Add("Scenario Selection",
        ui::Combo("##ScenarioSelection", selectedIndex, labels)
            .OnChanged([this, paths](int index) {
              if (index >= 0
                  && static_cast<std::size_t>(index) < paths.size()) {
                controller_.Load(paths[static_cast<std::size_t>(index)]);
              }
            }));
  }

  const int variantIndex = VariantIndex(model.executionVariant);
  fields.Add("Autopilot",
      ui::Combo("##ExecutionVariant", variantIndex, {"Baseline", "Primary"})
          .Tooltip("Execution Variant; the Scenario definition remains shared")
          .OnChanged([this](int index) {
            controller_.OnEvent(
                ExecutionVariantChanged{VariantFromIndex(index)});
          }));
  static_cast<ui::UIElement>(fields).Render();

  ImGui::TextDisabled("Directory: %s", model.directory.string().c_str());
  if (!model.statusMessage.empty()) {
    if (model.statusIsError) {
      static_cast<ui::UIElement>(
          ui::StatusBadge("Error", ui::StatusTone::Error))
          .Render();
      ImGui::SameLine();
    }
    ImGui::TextWrapped("%s", model.statusMessage.c_str());
  }
}

void ScenarioSetupPopup::DrawSummary() {
  const ScenarioFileModel &model = controller_.GetModel();
  const sim::SimScenario &scenario = model.draft;

  ImGui::SeparatorText("Scenario");
  ui::PropertyGridBuilder identity = MakeGrid("ScenarioSetupIdentity");
  identity.Add("Name / ID", ui::Text(scenario.name))
      .Add("Scenario Type", ui::Text(scenario.scenarioType))
      .Add("Aircraft", ui::Text(scenario.aircraft))
      .Add("Duration",
          ui::ValueLabel("##SetupDuration", scenario.durationSec, "%.3f s"))
      .Add("Time Step",
          ui::ValueLabel("##SetupTimeStep", scenario.dtSec, "%.6f s"));
  static_cast<ui::UIElement>(identity).Render();

  ImGui::SeparatorText("Initial Condition");
  const sim::InitialCondition &condition = scenario.initialCondition;
  char attitudeSummary[160]{};
  std::snprintf(attitudeSummary,
      sizeof(attitudeSummary),
      "roll %.3f deg, pitch %.3f deg, heading %.3f deg",
      math::RadToDeg(condition.rollRad),
      math::RadToDeg(condition.pitchRad),
      math::RadToDeg(condition.headingRad));
  ui::PropertyGridBuilder initial = MakeGrid("ScenarioSetupInitial");
  initial
      .Add("Altitude ASL",
          ui::ValueLabel(
              "##SetupAltitude", condition.altitudeAslM, "%.3f m"))
      .Add("CAS",
          ui::ValueLabel("##SetupAirspeed",
              condition.calibratedAirspeedMps,
              "%.3f m/s"))
      .Add("Attitude", ui::Text(attitudeSummary))
      .Add("Environment",
          ui::Text(scenario.windEnabled ? "Wind enabled" : "No wind"))
      .Add("Trim",
          ui::Text(std::string(scenario.runTrim ? "Enabled, " : "Disabled, ")
                   + TrimModeLabel(scenario.trimMode)));
  static_cast<ui::UIElement>(initial).Render();

  ImGui::SeparatorText("Events / Commands");
  if (scenario.events.empty()) {
    ui::TextDisabled("No events.").Render();
  } else {
    for (const sim::ScenarioEventDefinition &event : scenario.events) {
      char summary[128]{};
      std::snprintf(summary,
          sizeof(summary),
          "%.3f s   roll command = %.3f deg",
          event.timeSec,
          math::RadToDeg(event.command.rollRad));
      ui::Text(summary).Render();
    }
  }
}

void ScenarioSetupPopup::DrawActions(const sim::SimSnapshot &snapshot) {
  ImGui::Spacing();
  ImGui::Separator();
  const float buttonWidth = ui::Ui(120.0F);
  const float totalWidth = buttonWidth * 2.0F + ImGui::GetStyle().ItemSpacing.x;
  ImGui::SetCursorPosX(
      ImGui::GetCursorPosX()
      + std::max(0.0F, ImGui::GetContentRegionAvail().x - totalWidth));
  if (ImGui::Button("Cancel", ImVec2(buttonWidth, 0.0F))) {
    Cancel();
  }
  ImGui::SameLine();

  const bool isStopped =
      snapshot.status.executionState == sim::SimExecutionState::Stopped;
  ImGui::BeginDisabled(!isStopped);
  if (ImGui::Button("Apply Scenario", ImVec2(buttonWidth, 0.0F))
      && controller_.Apply()) {
    closeRequested_ = true;
  }
  ImGui::EndDisabled();
  if (!isStopped && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip("Stop the simulation before applying a Scenario.");
  }
}
} // namespace gui
