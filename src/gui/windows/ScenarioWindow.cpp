#include "gui/windows/ScenarioWindow.hpp"

#include "common/math/Math.hpp"
#include "flightui/FlightUI.hpp"
#include "sim/execution/ExecutionVariant.hpp"
#include "sim/runtime/SimContracts.hpp"

#include <imgui.h>

#include <cstdio>
#include <string>

namespace gui {

namespace {
constexpr float InitialWindowWidth = 430.0F;
constexpr float InitialWindowHeight = 620.0F;
constexpr float FieldLabelWidthRatio = 0.44F;
constexpr float MinimumFieldLabelWidth = 110.0F;
constexpr float MaximumFieldLabelWidth = 180.0F;
constexpr float MinimumTwoColumnWidth = 320.0F;

ui::PropertyGridBuilder MakeScenarioPropertyGrid(const char *id) {
  return ui::PropertyGrid(id)
      .LabelWidthRatio(FieldLabelWidthRatio)
      .MinimumLabelWidth(MinimumFieldLabelWidth)
      .MaximumLabelWidth(MaximumFieldLabelWidth)
      .SingleColumnThreshold(MinimumTwoColumnWidth)
      .AlternatingRows();
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

const char *CommandTypeLabel(sim::ScenarioCommandType type) {
  switch (type) {
  case sim::ScenarioCommandType::RollHold:
    return "Roll hold";
  }
  return "Unknown";
}

void DrawIdentity(const sim::ResolvedExecutionSpec &execution) {
  const sim::SimScenario &scenario = execution.scenario;
  ui::PropertyGridBuilder fields = MakeScenarioPropertyGrid("CurrentIdentity");
  fields.Add("Scenario", ui::Text(scenario.name))
      .Add("Scenario Type", ui::Text(scenario.scenarioType))
      .Add("Schema Version", ui::Text(std::to_string(scenario.schemaVersion)))
      .Add("Aircraft", ui::Text(scenario.aircraft))
      .Add("Autopilot", ui::Text(std::string(sim::ToString(execution.variant))))
      .Add("Duration",
          ui::ValueLabel("##Duration", scenario.durationSec, "%.3f s"))
      .Add("Time Step", ui::ValueLabel("##TimeStep", scenario.dtSec, "%.6f s"));
  static_cast<ui::UIElement>(fields).Render();
}

void DrawInitialCondition(const sim::InitialCondition &condition) {
  ImGui::SeparatorText("Initial Condition");
  ui::PropertyGridBuilder fields = MakeScenarioPropertyGrid("CurrentInitial");
  fields
      .Add("Latitude",
          ui::ValueLabel("##Latitude",
              math::RadToDeg(condition.latitudeRad),
              "%.3f deg"))
      .Add("Longitude",
          ui::ValueLabel("##Longitude",
              math::RadToDeg(condition.longitudeRad),
              "%.3f deg"))
      .Add("Altitude ASL",
          ui::ValueLabel("##Altitude", condition.altitudeAslM, "%.3f m"))
      .Add("CAS",
          ui::ValueLabel("##Airspeed",
              condition.calibratedAirspeedMps,
              "%.3f m/s"))
      .Add("Roll",
          ui::ValueLabel(
              "##Roll", math::RadToDeg(condition.rollRad), "%.3f deg"))
      .Add("Pitch",
          ui::ValueLabel(
              "##Pitch", math::RadToDeg(condition.pitchRad), "%.3f deg"))
      .Add("Heading",
          ui::ValueLabel("##Heading",
              math::RadToDeg(condition.headingRad),
              "%.3f deg"));
  static_cast<ui::UIElement>(fields).Render();
}

void DrawConditions(const sim::SimScenario &scenario) {
  ImGui::SeparatorText("Conditions");
  ui::PropertyGridBuilder fields =
      MakeScenarioPropertyGrid("CurrentConditions");
  fields.Add("Wind", ui::Text(scenario.windEnabled ? "Enabled" : "Disabled"))
      .Add("Trim", ui::Text(scenario.runTrim ? "Enabled" : "Disabled"))
      .Add("Trim Mode", ui::Text(TrimModeLabel(scenario.trimMode)));
  static_cast<ui::UIElement>(fields).Render();
}

void DrawEvents(const sim::SimScenario &scenario) {
  ImGui::SeparatorText("Events");
  if (scenario.events.empty()) {
    ui::TextDisabled("No events.").Render();
    return;
  }

  for (const sim::ScenarioEventDefinition &event : scenario.events) {
    char summary[160]{};
    std::snprintf(summary,
        sizeof(summary),
        "%.3f s   %s = %.3f deg",
        event.timeSec,
        CommandTypeLabel(event.command.type),
        math::RadToDeg(event.command.rollRad));
    ui::Text(summary).Render();
  }
}

void DrawAcceptance(const sim::SimScenario &scenario) {
  ImGui::SeparatorText("Acceptance Criteria");
  ui::PropertyGridBuilder fields =
      MakeScenarioPropertyGrid("CurrentAcceptance");
  fields
      .Add("Settling Band",
          ui::ValueLabel("##SettlingBand",
              math::RadToDeg(scenario.settlingBandRad),
              "%.3f deg"))
      .Add("Settling Limit",
          ui::ValueLabel("##SettlingLimit",
              scenario.settlingTimeLimitSec,
              "%.3f s"))
      .Add("Overshoot Limit",
          ui::ValueLabel("##Overshoot",
              math::RadToDeg(scenario.overshootLimitRad),
              "%.3f deg"))
      .Add("Oscillation Cycles",
          ui::ValueLabel("##Oscillation",
              scenario.maxOscillationCycles,
              "%.3f"));
  static_cast<ui::UIElement>(fields).Render();
}
} // namespace

ScenarioWindow::ScenarioWindow()
    : Window("Current Scenario", editor_icon_aliases::Scenario, "Scenario") {}

void ScenarioWindow::PrepareWindow() {
  ImGui::SetNextWindowSize(
      ImVec2(ui::Ui(InitialWindowWidth), ui::Ui(InitialWindowHeight)),
      ImGuiCond_FirstUseEver);
}

void ScenarioWindow::OnRender(const sim::SimSnapshot &snapshot) {
  ui::TextDisabled("Resolved values from the Scenario applied to the runtime.")
      .Render();
  ImGui::Spacing();
  if (!snapshot.appliedExecution.has_value()) {
    ui::TextDisabled("No Scenario is currently applied.").Render();
    return;
  }

  const sim::ResolvedExecutionSpec &execution = *snapshot.appliedExecution;
  DrawIdentity(execution);
  DrawInitialCondition(execution.scenario.initialCondition);
  DrawConditions(execution.scenario);
  DrawEvents(execution.scenario);
  DrawAcceptance(execution.scenario);

  if (!execution.source.file.empty()
      || !execution.source.digestSha256.empty()) {
    ImGui::SeparatorText("Source");
    ui::PropertyGridBuilder fields = MakeScenarioPropertyGrid("CurrentSource");
    fields.Add("File",
        ui::Text(execution.source.file.empty() ? "Embedded"
                                               : execution.source.file));
    if (!execution.source.digestSha256.empty()) {
      fields.Add("SHA-256", ui::Text(execution.source.digestSha256));
    }
    static_cast<ui::UIElement>(fields).Render();
  }
}
} // namespace gui
