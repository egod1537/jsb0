#include "application/gui/windows/ScenarioWindow.hpp"

#include "application/gui/GUI.hpp"
#include "application/sim/scenario/SimulationScenarioSerializer.hpp"
#include "flightui/core/UIScale.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <string>
#include <utility>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float InitialWindowWidth = 430.0F;
constexpr float InitialWindowHeight = 680.0F;
constexpr float FieldLabelWidthRatio = 0.44F;
constexpr float MinimumFieldLabelWidth = 110.0F;
constexpr float MaximumFieldLabelWidth = 180.0F;
constexpr float MinimumTwoColumnWidth = 320.0F;
constexpr std::size_t ScenarioNameCapacity = 256;
constexpr const char *DefaultScenarioFileName = "c172_roll_hold_5deg.yaml";

constexpr ImGuiTableFlags FieldTableFlags = ImGuiTableFlags_SizingStretchProp
                                            | ImGuiTableFlags_NoSavedSettings
                                            | ImGuiTableFlags_PadOuterX;

float CalculateFieldLabelWidth() {
  return std::clamp(ImGui::GetContentRegionAvail().x * FieldLabelWidthRatio,
      UI::Ui(MinimumFieldLabelWidth),
      UI::Ui(MaximumFieldLabelWidth));
}

bool BeginFieldTable(const char *id) {
  const bool useTwoColumns =
      ImGui::GetContentRegionAvail().x >= UI::Ui(MinimumTwoColumnWidth);
  const int columnCount = useTwoColumns ? 2 : 1;
  if (!ImGui::BeginTable(id, columnCount, FieldTableFlags)) {
    return false;
  }
  if (useTwoColumns) {
    ImGui::TableSetupColumn("Label",
        ImGuiTableColumnFlags_WidthFixed,
        CalculateFieldLabelWidth());
    ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
  } else {
    ImGui::TableSetupColumn("Field", ImGuiTableColumnFlags_WidthStretch);
  }
  return true;
}

void DrawFieldLabel(const char *label) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  if (ImGui::TableGetColumnCount() == 1) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
  } else {
    ImGui::TableSetColumnIndex(1);
  }
  ImGui::SetNextItemWidth(-1.0F);
  ImGui::PushID(label);
}

void EndField() { ImGui::PopID(); }

void DrawDoubleField(const char *label, double &value, double step,
    double fastStep) {
  DrawFieldLabel(label);
  ImGui::InputDouble("##Value", &value, step, fastStep, "%.2f");
  EndField();
}

void DrawBooleanField(const char *label, bool &value) {
  DrawFieldLabel(label);
  ImGui::Checkbox("##Value", &value);
  EndField();
}

int TrimModeIndex(gnc::TrimMode mode) {
  switch (mode) {
  case gnc::TrimMode::Longitudinal:
    return 0;
  case gnc::TrimMode::Full:
    return 1;
  case gnc::TrimMode::Ground:
    return 2;
  }
  return 0;
}

gnc::TrimMode TrimModeFromIndex(int index) {
  switch (index) {
  case 1:
    return gnc::TrimMode::Full;
  case 2:
    return gnc::TrimMode::Ground;
  case 0:
  default:
    return gnc::TrimMode::Longitudinal;
  }
}

std::filesystem::path FindDefaultScenarioDirectory() {
  std::error_code error;
  std::filesystem::path directory = std::filesystem::current_path(error);
  if (error) {
    return std::filesystem::path("scenarios");
  }

  while (!directory.empty()) {
    if (std::filesystem::exists(directory / "CMakeLists.txt", error) && !error
        && std::filesystem::exists(directory / "src", error) && !error) {
      return directory / "scenarios";
    }
    const std::filesystem::path parent = directory.parent_path();
    if (parent == directory) {
      break;
    }
    directory = parent;
  }
  return std::filesystem::current_path() / "scenarios";
}

bool HasYamlExtension(const std::filesystem::path &path) {
  std::string extension = path.extension().string();
  std::transform(extension.begin(),
      extension.end(),
      extension.begin(),
      [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
      });
  return extension == ".yaml" || extension == ".yml";
}

bool BeginScenarioFoldOut(const char *label, const char *id, bool &open) {
  ImGui::SetNextItemOpen(open, ImGuiCond_Always);
  const std::string treeLabel = std::string(label) + "###" + id;
  open = ImGui::CollapsingHeader(treeLabel.c_str(),
      ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_SpanAvailWidth);
  return open;
}
} // namespace

ScenarioWindow::ScenarioWindow(std::filesystem::path scenarioDirectory)
    : Window("Scenario", EditorIconAliases::Scenario),
      scenarioDirectory_(scenarioDirectory.empty()
                             ? FindDefaultScenarioDirectory()
                             : std::move(scenarioDirectory)) {
  SetFileNameInput(DefaultScenarioFileName);
}

sim::SimulationScenario &ScenarioWindow::GetScenario() { return scenario_; }

const sim::SimulationScenario &ScenarioWindow::GetScenario() const {
  return scenario_;
}

void ScenarioWindow::ResetDefaults() { scenario_ = sim::SimulationScenario{}; }

void ScenarioWindow::NewScenario() {
  scenario_ = sim::SimulationScenario{};
  cleanScenario_ = scenario_;
  currentFilePath_.clear();
  SetFileNameInput("untitled.yaml");
  SetFileStatus("Created a new scenario. Use Save As to persist it.", false);
}

bool ScenarioWindow::LoadScenarioFile(const std::filesystem::path &path) {
  const std::filesystem::path resolvedPath = ResolveScenarioPath(path);
  sim::SimulationScenario loadedScenario;
  std::string error;
  if (!sim::SimulationScenarioSerializer::Load(resolvedPath,
          loadedScenario,
          error)) {
    SetFileStatus(std::move(error), true);
    return false;
  }

  scenario_ = std::move(loadedScenario);
  cleanScenario_ = scenario_;
  currentFilePath_ = resolvedPath;
  SetFileNameInput(resolvedPath.filename());
  SetFileStatus("Loaded " + resolvedPath.filename().string(), false);
  return true;
}

bool ScenarioWindow::SaveScenarioFile() {
  if (currentFilePath_.empty()) {
    SetFileStatus("No scenario file is connected. Use Save As.", true);
    return false;
  }
  return SaveScenarioFileAs(currentFilePath_);
}

bool ScenarioWindow::SaveScenarioFileAs(const std::filesystem::path &path) {
  const std::filesystem::path resolvedPath = ResolveScenarioPath(path);
  std::string error;
  if (!sim::SimulationScenarioSerializer::Save(resolvedPath,
          scenario_,
          error)) {
    SetFileStatus(std::move(error), true);
    return false;
  }

  cleanScenario_ = scenario_;
  currentFilePath_ = resolvedPath;
  SetFileNameInput(resolvedPath.filename());
  SetFileStatus("Saved " + resolvedPath.filename().string(), false);
  return true;
}

bool ScenarioWindow::IsDirty() const { return scenario_ != cleanScenario_; }

const std::filesystem::path &ScenarioWindow::GetScenarioDirectory() const {
  return scenarioDirectory_;
}

const std::filesystem::path &ScenarioWindow::GetCurrentFilePath() const {
  return currentFilePath_;
}

const std::string &ScenarioWindow::GetFileStatusMessage() const {
  return fileStatusMessage_;
}

void ScenarioWindow::PrepareWindow() {
  ImGui::SetNextWindowSize(
      ImVec2(UI::Ui(InitialWindowWidth), UI::Ui(InitialWindowHeight)),
      ImGuiCond_FirstUseEver);
}

void ScenarioWindow::OnRender(gui::GUI &gui) {
  DrawFileSection();
  DrawScenarioSection();
  DrawInitialConditionSection();
  DrawEnvironmentSection();
  DrawTrimSection();
  DrawCommandSection();
  DrawSimulationSection();
  DrawAcceptanceCriteriaSection();
  DrawActions(gui);
}

void ScenarioWindow::DrawFileSection() {
  ImGui::SeparatorText("Scenario File");
  const std::string connectionLabel =
      currentFilePath_.empty() ? "Not connected"
                               : currentFilePath_.filename().string();
  ImGui::Text("%s%s", connectionLabel.c_str(), IsDirty() ? " *" : "");
  ImGui::SetNextItemWidth(-1.0F);
  ImGui::InputText("##ScenarioFileName",
      fileNameInput_.data(),
      fileNameInput_.size());
  ImGui::TextDisabled("Directory: %s", scenarioDirectory_.string().c_str());

  if (ImGui::BeginTable("ScenarioFileActions",
          4,
          ImGuiTableFlags_SizingStretchSame
              | ImGuiTableFlags_NoSavedSettings)) {
    ImGui::TableNextColumn();
    if (ImGui::Button("New", ImVec2(-1.0F, 0.0F))) {
      NewScenario();
    }

    const bool hasFileName = fileNameInput_[0] != '\0';
    ImGui::TableNextColumn();
    ImGui::BeginDisabled(!hasFileName);
    if (ImGui::Button("Load", ImVec2(-1.0F, 0.0F))) {
      std::filesystem::path path;
      if (ResolveFileNameInput(path)) {
        LoadScenarioFile(path);
      }
    }
    ImGui::EndDisabled();

    ImGui::TableNextColumn();
    ImGui::BeginDisabled(currentFilePath_.empty());
    if (ImGui::Button("Save", ImVec2(-1.0F, 0.0F))) {
      SaveScenarioFile();
    }
    ImGui::EndDisabled();

    ImGui::TableNextColumn();
    ImGui::BeginDisabled(!hasFileName);
    if (ImGui::Button("Save As", ImVec2(-1.0F, 0.0F))) {
      std::filesystem::path path;
      if (ResolveFileNameInput(path)) {
        SaveScenarioFileAs(path);
      }
    }
    ImGui::EndDisabled();
    ImGui::EndTable();
  }

  if (!fileStatusMessage_.empty()) {
    if (fileStatusIsError_) {
      ImGui::PushStyleColor(ImGuiCol_Text,
          ImGui::GetStyleColorVec4(ImGuiCol_PlotLinesHovered));
    }
    ImGui::TextWrapped("%s", fileStatusMessage_.c_str());
    if (fileStatusIsError_) {
      ImGui::PopStyleColor();
    }
  }
  ImGui::Spacing();
}

std::filesystem::path ScenarioWindow::ResolveScenarioPath(
    const std::filesystem::path &path) const {
  std::filesystem::path resolved =
      path.is_absolute() ? path : scenarioDirectory_ / path;
  if (resolved.extension().empty()) {
    resolved.replace_extension(".yaml");
  }
  return resolved.lexically_normal();
}

bool ScenarioWindow::ResolveFileNameInput(std::filesystem::path &path) {
  const std::filesystem::path input(fileNameInput_.data());
  if (input.empty()) {
    SetFileStatus("Enter a scenario file name.", true);
    return false;
  }
  if (input.has_parent_path()) {
    SetFileStatus("Scenario File accepts a file name, not a directory path.",
        true);
    return false;
  }

  path = input;
  if (path.extension().empty()) {
    path.replace_extension(".yaml");
  }
  if (!HasYamlExtension(path)) {
    SetFileStatus("Scenario file extension must be .yaml or .yml.", true);
    return false;
  }
  return true;
}

void ScenarioWindow::SetFileNameInput(const std::filesystem::path &path) {
  std::snprintf(fileNameInput_.data(),
      fileNameInput_.size(),
      "%s",
      path.filename().string().c_str());
}

void ScenarioWindow::SetFileStatus(std::string message, bool error) {
  fileStatusMessage_ = std::move(message);
  fileStatusIsError_ = error;
}

void ScenarioWindow::DrawScenarioSection() {
  if (!BeginScenarioFoldOut("Scenario",
          "ScenarioSection",
          scenarioSectionOpen_)) {
    return;
  }
  if (!BeginFieldTable("ScenarioFields")) {
    return;
  }

  std::array<char, ScenarioNameCapacity> nameBuffer{};
  std::snprintf(nameBuffer.data(),
      nameBuffer.size(),
      "%s",
      scenario_.name.c_str());
  DrawFieldLabel("Name");
  if (ImGui::InputText("##Value", nameBuffer.data(), nameBuffer.size())) {
    scenario_.name = nameBuffer.data();
  }
  EndField();
  ImGui::EndTable();
}

void ScenarioWindow::DrawInitialConditionSection() {
  if (!BeginScenarioFoldOut("Initial Condition",
          "InitialConditionSection",
          initialConditionSectionOpen_)) {
    return;
  }
  if (!BeginFieldTable("InitialConditionFields")) {
    return;
  }
  DrawDoubleField("Altitude [ft]", scenario_.altitudeFt, 100.0, 1000.0);
  DrawDoubleField("Airspeed [kt]", scenario_.airspeedKts, 1.0, 10.0);
  DrawDoubleField("Roll [deg]", scenario_.initialRollDeg, 0.1, 1.0);
  DrawDoubleField("Pitch [deg]", scenario_.initialPitchDeg, 0.1, 1.0);
  DrawDoubleField("Heading [deg]", scenario_.initialHeadingDeg, 1.0, 10.0);
  ImGui::EndTable();
}

void ScenarioWindow::DrawEnvironmentSection() {
  if (!BeginScenarioFoldOut("Environment",
          "EnvironmentSection",
          environmentSectionOpen_)) {
    return;
  }
  if (!BeginFieldTable("EnvironmentFields")) {
    return;
  }
  DrawBooleanField("Wind Enabled", scenario_.windEnabled);
  ImGui::EndTable();
}

void ScenarioWindow::DrawTrimSection() {
  if (!BeginScenarioFoldOut("Trim", "TrimSection", trimSectionOpen_)) {
    return;
  }
  if (!BeginFieldTable("TrimFields")) {
    return;
  }
  DrawBooleanField("Run Trim", scenario_.runTrim);

  int trimModeIndex = TrimModeIndex(scenario_.trimMode);
  DrawFieldLabel("Trim Mode");
  constexpr std::array TrimModeLabels{"Longitudinal", "Full", "Ground"};
  if (ImGui::Combo("##Value",
          &trimModeIndex,
          TrimModeLabels.data(),
          static_cast<int>(TrimModeLabels.size()))) {
    scenario_.trimMode = TrimModeFromIndex(trimModeIndex);
  }
  EndField();
  ImGui::EndTable();
}

void ScenarioWindow::DrawCommandSection() {
  if (!BeginScenarioFoldOut("Command", "CommandSection", commandSectionOpen_)) {
    return;
  }
  if (!BeginFieldTable("CommandFields")) {
    return;
  }
  DrawDoubleField("Command Start [s]", scenario_.commandStartSec, 0.1, 1.0);
  DrawDoubleField("Commanded Roll [deg]", scenario_.commandedRollDeg, 0.1, 1.0);
  ImGui::EndTable();
}

void ScenarioWindow::DrawSimulationSection() {
  if (!BeginScenarioFoldOut("Simulation",
          "SimulationSection",
          simulationSectionOpen_)) {
    return;
  }
  if (!BeginFieldTable("SimulationFields")) {
    return;
  }
  DrawDoubleField("Duration [s]", scenario_.durationSec, 1.0, 10.0);
  ImGui::EndTable();
}

void ScenarioWindow::DrawAcceptanceCriteriaSection() {
  if (!BeginScenarioFoldOut("Acceptance Criteria",
          "AcceptanceCriteriaSection",
          acceptanceSectionOpen_)) {
    return;
  }
  if (!BeginFieldTable("AcceptanceCriteriaFields")) {
    return;
  }
  DrawDoubleField("Settling Band [deg]", scenario_.settlingBandDeg, 0.1, 0.5);
  DrawDoubleField("Settling Time Limit [s]",
      scenario_.settlingTimeLimitSec,
      0.5,
      1.0);
  DrawDoubleField("Overshoot Limit [deg]",
      scenario_.overshootLimitDeg,
      0.1,
      0.5);
  DrawDoubleField("Max Oscillation Cycles",
      scenario_.maxOscillationCycles,
      0.5,
      1.0);
  ImGui::EndTable();
}

void ScenarioWindow::DrawActions(gui::GUI &gui) {
  ImGui::Spacing();
  ImGui::Separator();
  const ImGuiStyle &style = ImGui::GetStyle();
  const float resetButtonWidth =
      ImGui::CalcTextSize("Reset Defaults").x + style.FramePadding.x * 2.0F;
  const float runButtonWidth =
      ImGui::CalcTextSize("Run Scenario").x + style.FramePadding.x * 2.0F;
  const bool stackButtons =
      ImGui::GetContentRegionAvail().x
      < resetButtonWidth + style.ItemSpacing.x + runButtonWidth;
  const ImVec2 buttonSize(stackButtons ? -1.0F : 0.0F, 0.0F);

  if (ImGui::Button("Reset Defaults", buttonSize)) {
    ResetDefaults();
  }

  if (!stackButtons) {
    ImGui::SameLine();
  }
  auto &executionControl = gui.GetSimulationExecutionControl();
  const bool isStopped = executionControl.GetSimulationExecutionState()
                         == application::SimulationExecutionState::Stopped;
  std::string validationError;
  const bool scenarioValid =
      sim::ValidateSimulationScenario(scenario_, &validationError);
  const bool canRun = isStopped && scenarioValid;
  ImGui::BeginDisabled(!canRun);
  if (ImGui::Button("Run Scenario", buttonSize)) {
    executionControl.RunScenario(scenario_);
  }
  ImGui::EndDisabled();
  if (!canRun && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip("%s",
        isStopped ? validationError.c_str()
                  : "Stop the simulation before running a scenario.");
  }
}
} // namespace gui
