#include "gui/windows/ScenarioWindow.hpp"

#include "flightui/FlightUI.hpp"

#include <imgui.h>

#include <array>
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

UI::PropertyGridBuilder MakeScenarioPropertyGrid(const char *id) {
  return UI::PropertyGrid(id)
      .LabelWidthRatio(FieldLabelWidthRatio)
      .MinimumLabelWidth(MinimumFieldLabelWidth)
      .MaximumLabelWidth(MaximumFieldLabelWidth)
      .SingleColumnThreshold(MinimumTwoColumnWidth);
}

UI::UIElement MakeDoubleField(const char *id, double &value, double step,
    double fastStep) {
  return UI::InputDouble(id, value)
      .Step(step)
      .FastStep(fastStep)
      .Format("%.2f")
      .OnChanged([&value](double changedValue) { value = changedValue; });
}

UI::UIElement MakeBooleanField(const char *id, bool &value) {
  return UI::Toggle(id, value).OnChanged(
      [&value](bool changedValue) { value = changedValue; });
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

} // namespace

ScenarioWindow::ScenarioWindow(std::filesystem::path scenarioDirectory)
    : Window("Scenario", EditorIconAliases::Scenario),
      ownedController_(
          std::make_unique<ScenarioController>(std::move(scenarioDirectory))),
      controller_(ownedController_.get()) {
  SetFileNameInput(controller_->GetModel().suggestedFileName);
}

ScenarioWindow::ScenarioWindow(SimulationController &controller,
    std::filesystem::path scenarioDirectory)
    : Window("Scenario", EditorIconAliases::Scenario),
      ownedController_(
          std::make_unique<ScenarioController>(std::move(scenarioDirectory),
              architecture::EventSink<ScenarioLaunchRequested>{
                  [&controller](const ScenarioLaunchRequested &event) {
                    controller.Handle(event);
                  }})),
      controller_(ownedController_.get()) {
  SetFileNameInput(controller_->GetModel().suggestedFileName);
}

sim::SimulationScenario &ScenarioWindow::GetScenario() {
  return controller_->EditDraftForCompatibility();
}

const sim::SimulationScenario &ScenarioWindow::GetScenario() const {
  return controller_->GetModel().draft;
}

void ScenarioWindow::ResetDefaults() {
  controller_->ResetDefaults();
  renderDraft_ = controller_->GetModel().draft;
}

void ScenarioWindow::NewScenario() {
  controller_->NewScenario();
  renderDraft_ = controller_->GetModel().draft;
  SetFileNameInput(controller_->GetModel().suggestedFileName);
}

bool ScenarioWindow::LoadScenarioFile(const std::filesystem::path &path) {
  const bool loaded = controller_->Load(path);
  if (loaded) {
    renderDraft_ = controller_->GetModel().draft;
    SetFileNameInput(controller_->GetModel().suggestedFileName);
  }
  return loaded;
}

bool ScenarioWindow::SaveScenarioFile() {
  const bool saved = controller_->Save();
  renderDraft_ = controller_->GetModel().draft;
  return saved;
}

bool ScenarioWindow::SaveScenarioFileAs(const std::filesystem::path &path) {
  const bool saved = controller_->SaveAs(path);
  if (saved) {
    renderDraft_ = controller_->GetModel().draft;
    SetFileNameInput(controller_->GetModel().suggestedFileName);
  }
  return saved;
}

bool ScenarioWindow::IsDirty() const { return controller_->IsDirty(); }

const std::filesystem::path &ScenarioWindow::GetScenarioDirectory() const {
  return controller_->GetModel().directory;
}

const std::filesystem::path &ScenarioWindow::GetCurrentFilePath() const {
  return controller_->GetModel().currentFilePath;
}

const std::string &ScenarioWindow::GetFileStatusMessage() const {
  return controller_->GetModel().statusMessage;
}

void ScenarioWindow::PrepareWindow() {
  ImGui::SetNextWindowSize(
      ImVec2(UI::Ui(InitialWindowWidth), UI::Ui(InitialWindowHeight)),
      ImGuiCond_FirstUseEver);
}

void ScenarioWindow::OnRender(const sim::SimulationSnapshot &snapshot) {
  renderDraft_ = controller_->GetModel().draft;
  DrawFileSection();
  DrawScenarioSection();
  DrawInitialConditionSection();
  DrawEnvironmentSection();
  DrawTrimSection();
  DrawCommandSection();
  DrawSimulationSection();
  DrawAcceptanceCriteriaSection();
  DrawActions(snapshot);
  if (renderDraft_ != controller_->GetModel().draft) {
    controller_->Handle(ScenarioDraftChanged{renderDraft_});
  }
}

void ScenarioWindow::DrawFileSection() {
  const ScenarioFileModel &file = controller_->GetModel();
  ImGui::SeparatorText("Scenario File");
  const std::string connectionLabel =
      file.currentFilePath.empty() ? "Not connected"
                                   : file.currentFilePath.filename().string();
  ImGui::Text("%s%s", connectionLabel.c_str(), IsDirty() ? " *" : "");
  ImGui::SetNextItemWidth(-1.0F);
  ImGui::InputText("##ScenarioFileName",
      fileNameInput_.data(),
      fileNameInput_.size());
  ImGui::TextDisabled("Directory: %s", file.directory.string().c_str());

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
    ImGui::BeginDisabled(file.currentFilePath.empty());
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

  if (!file.statusMessage.empty()) {
    if (file.statusIsError) {
      const UI::UIElement badge =
          UI::StatusBadge("Error", UI::StatusTone::Error);
      badge.Render();
      ImGui::SameLine();
    }
    ImGui::TextWrapped("%s", file.statusMessage.c_str());
  }
  ImGui::Spacing();
}

bool ScenarioWindow::ResolveFileNameInput(std::filesystem::path &path) {
  return controller_->ResolveFileName(fileNameInput_.data(), path);
}

void ScenarioWindow::SetFileNameInput(const std::filesystem::path &path) {
  std::snprintf(fileNameInput_.data(),
      fileNameInput_.size(),
      "%s",
      path.filename().string().c_str());
}

void ScenarioWindow::DrawScenarioSection() {
  UI::PropertyGridBuilder fields = MakeScenarioPropertyGrid("ScenarioFields");
  fields.Add(UI::PropertyRow("Name")[UI::Custom([this] {
    std::array<char, ScenarioNameCapacity> nameBuffer{};
    std::snprintf(nameBuffer.data(),
        nameBuffer.size(),
        "%s",
        renderDraft_.name.c_str());
    ImGui::SetNextItemWidth(-1.0F);
    if (ImGui::InputText("##Value", nameBuffer.data(), nameBuffer.size())) {
      renderDraft_.name = nameBuffer.data();
    }
  })]);
  UI::FoldOut("Scenario")
      .Open(scenarioSectionOpen_)
      .Flags(ImGuiTreeNodeFlags_CollapsingHeader
             | ImGuiTreeNodeFlags_SpanAvailWidth)
      .Id("ScenarioSection")[fields]
      .Render();
}

void ScenarioWindow::DrawInitialConditionSection() {
  UI::PropertyGridBuilder fields =
      MakeScenarioPropertyGrid("InitialConditionFields");
  fields
      .Add(UI::PropertyRow("Altitude [ft]")[MakeDoubleField("Altitude",
          renderDraft_.altitudeFt,
          100.0,
          1000.0)])
      .Add(UI::PropertyRow("Airspeed [kt]")[MakeDoubleField("Airspeed",
          renderDraft_.airspeedKts,
          1.0,
          10.0)])
      .Add(UI::PropertyRow("Roll [deg]")[MakeDoubleField("InitialRoll",
          renderDraft_.initialRollDeg,
          0.1,
          1.0)])
      .Add(UI::PropertyRow("Pitch [deg]")[MakeDoubleField("InitialPitch",
          renderDraft_.initialPitchDeg,
          0.1,
          1.0)])
      .Add(UI::PropertyRow("Heading [deg]")[MakeDoubleField("InitialHeading",
          renderDraft_.initialHeadingDeg,
          1.0,
          10.0)]);
  UI::FoldOut("Initial Condition")
      .Open(initialConditionSectionOpen_)
      .Flags(ImGuiTreeNodeFlags_CollapsingHeader
             | ImGuiTreeNodeFlags_SpanAvailWidth)
      .Id("InitialConditionSection")[fields]
      .Render();
}

void ScenarioWindow::DrawEnvironmentSection() {
  UI::PropertyGridBuilder fields =
      MakeScenarioPropertyGrid("EnvironmentFields");
  fields.Add(UI::PropertyRow("Wind Enabled")[MakeBooleanField("WindEnabled",
      renderDraft_.windEnabled)]);
  UI::FoldOut("Environment")
      .Open(environmentSectionOpen_)
      .Flags(ImGuiTreeNodeFlags_CollapsingHeader
             | ImGuiTreeNodeFlags_SpanAvailWidth)
      .Id("EnvironmentSection")[fields]
      .Render();
}

void ScenarioWindow::DrawTrimSection() {
  int trimModeIndex = TrimModeIndex(renderDraft_.trimMode);
  UI::PropertyGridBuilder fields = MakeScenarioPropertyGrid("TrimFields");
  fields
      .Add(UI::PropertyRow(
          "Run Trim")[MakeBooleanField("RunTrim", renderDraft_.runTrim)])
      .Add(UI::PropertyRow("Trim Mode")[UI::Combo("TrimMode",
          trimModeIndex,
          {"Longitudinal", "Full", "Ground"})
              .OnChanged([this](int index) {
                renderDraft_.trimMode = TrimModeFromIndex(index);
              })]);
  UI::FoldOut("Trim")
      .Open(trimSectionOpen_)
      .Flags(ImGuiTreeNodeFlags_CollapsingHeader
             | ImGuiTreeNodeFlags_SpanAvailWidth)
      .Id("TrimSection")[fields]
      .Render();
}

void ScenarioWindow::DrawCommandSection() {
  UI::PropertyGridBuilder fields = MakeScenarioPropertyGrid("CommandFields");
  fields
      .Add(UI::PropertyRow("Command Start [s]")[MakeDoubleField("CommandStart",
          renderDraft_.commandStartSec,
          0.1,
          1.0)])
      .Add(UI::PropertyRow(
          "Commanded Roll [deg]")[MakeDoubleField("CommandedRoll",
          renderDraft_.commandedRollDeg,
          0.1,
          1.0)]);
  UI::FoldOut("Command")
      .Open(commandSectionOpen_)
      .Flags(ImGuiTreeNodeFlags_CollapsingHeader
             | ImGuiTreeNodeFlags_SpanAvailWidth)
      .Id("CommandSection")[fields]
      .Render();
}

void ScenarioWindow::DrawSimulationSection() {
  UI::PropertyGridBuilder fields = MakeScenarioPropertyGrid("SimulationFields");
  fields.Add(UI::PropertyRow("Duration [s]")
          [MakeDoubleField("Duration", renderDraft_.durationSec, 1.0, 10.0)]);
  UI::FoldOut("Simulation")
      .Open(simulationSectionOpen_)
      .Flags(ImGuiTreeNodeFlags_CollapsingHeader
             | ImGuiTreeNodeFlags_SpanAvailWidth)
      .Id("SimulationSection")[fields]
      .Render();
}

void ScenarioWindow::DrawAcceptanceCriteriaSection() {
  UI::PropertyGridBuilder fields =
      MakeScenarioPropertyGrid("AcceptanceCriteriaFields");
  fields
      .Add(
          UI::PropertyRow("Settling Band [deg]")[MakeDoubleField("SettlingBand",
              renderDraft_.settlingBandDeg,
              0.1,
              0.5)])
      .Add(UI::PropertyRow(
          "Settling Time Limit [s]")[MakeDoubleField("SettlingTimeLimit",
          renderDraft_.settlingTimeLimitSec,
          0.5,
          1.0)])
      .Add(UI::PropertyRow(
          "Overshoot Limit [deg]")[MakeDoubleField("OvershootLimit",
          renderDraft_.overshootLimitDeg,
          0.1,
          0.5)])
      .Add(UI::PropertyRow(
          "Max Oscillation Cycles")[MakeDoubleField("MaxOscillationCycles",
          renderDraft_.maxOscillationCycles,
          0.5,
          1.0)]);
  UI::FoldOut("Acceptance Criteria")
      .Open(acceptanceSectionOpen_)
      .Flags(ImGuiTreeNodeFlags_CollapsingHeader
             | ImGuiTreeNodeFlags_SpanAvailWidth)
      .Id("AcceptanceCriteriaSection")[fields]
      .Render();
}

void ScenarioWindow::DrawActions(const sim::SimulationSnapshot &snapshot) {
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
  const bool isStopped =
      snapshot.status.executionState == sim::SimulationExecutionState::Stopped;
  std::string validationError;
  const bool scenarioValid =
      sim::ValidateSimulationScenario(renderDraft_, &validationError);
  const bool canRun = controller_ != nullptr && isStopped && scenarioValid;
  ImGui::BeginDisabled(!canRun);
  if (ImGui::Button("Run Scenario", buttonSize)) {
    controller_->Handle(ScenarioLaunchRequested{renderDraft_});
  }
  ImGui::EndDisabled();
  if (!canRun && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip("%s",
        isStopped ? validationError.c_str()
                  : "Stop the simulation before running a scenario.");
  }
}
} // namespace gui
