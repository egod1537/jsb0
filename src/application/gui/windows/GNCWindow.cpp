#include "GNCWindow.hpp"

#include "application/gui/GUI.hpp"
#include "application/gui/panels/AutopilotPanel.hpp"
#include "application/gui/panels/BaselineAutopilotPanel.hpp"
#include "application/gui/panels/ManualControlPanel.hpp"
#include "application/gui/panels/TrimPanel.hpp"
#include "application/sim/Aircraft.hpp"
#include "application/sim/Simulation.hpp"
#include "application/sim/control/FlightControlMode.hpp"
#include "application/sim/gnc/autopilot/MyAutopilot.hpp"
#include "application/sim/gnc/autopilot/PX4Autopilot.hpp"
#include "application/sim/gnc/hold/Px4RollHoldReferenceController.hpp"
#include "common/math/Math.hpp"
#include "flightui/FlightUI.hpp"

#include <algorithm>
#include <iostream>
#include <string>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float AutopilotSelectorHeight = 32.0F;
constexpr float AutopilotSelectorSpacing = 4.0F;
constexpr float AutopilotSelectorRounding = 4.0F;
constexpr float AutopilotIdentityHeight = 34.0F;
constexpr float AutopilotIdentityAccentWidth = 3.0F;
constexpr float AutopilotIdentityPadding = 12.0F;

gnc::RollHoldSettings MakeRollHoldSettings(const AutopilotPanelState &state) {
  return {
      .targetRollRad = math::DegToRad(state.rollTargetDeg),
      .dampingRatio = state.rollHoldDampingRatio,
      .naturalFrequencyRadPerSec = state.rollHoldNaturalFrequencyRadPerSec,
  };
}

void ApplyBaselinePx4RollSettings(gnc::PX4Autopilot &autopilot,
    const BaselineAutopilotPanelState &state) {
  gnc::Px4RollHoldReferenceSettings settings = autopilot.GetRollHoldSettings();
  settings.timeConstantSec = state.px4RollTimeConstantSec;
  settings.maximumRollRateRadPerSec =
      math::DegToRad(state.px4RollMaximumRateDegPerSec);
  settings.rateProportionalGain = state.px4RollRateProportionalGain;
  settings.rateIntegralGain = state.px4RollRateIntegralGain;
  settings.rateDerivativeGain = state.px4RollRateDerivativeGain;
  settings.rateFeedForwardGain = state.px4RollRateFeedForwardGain;
  settings.integratorLimit = state.px4RollIntegratorLimit;
  autopilot.SetRollHoldSettings(settings);
}

bool HasAnyAutopilotControlEnabled(const AutopilotPanelState &state) {
  return state.rollHold;
}

bool IsBaselineAutopilotAvailable(const GUI &gui) {
  const sim::Simulation *baseline = gui.GetBaselineSimulation();
  return baseline != nullptr && baseline->IsInitialized();
}

sim::Simulation &GetAutopilotSimulation(GUI &gui,
    AutopilotSelection selection) {
  if (selection == AutopilotSelection::Baseline) {
    sim::Simulation *baseline = gui.GetBaselineSimulation();
    if (baseline != nullptr && baseline->IsInitialized()) {
      return *baseline;
    }
  }
  return gui.GetPrimarySimulation();
}

bool DrawAutopilotSegment(const char *label, bool selected, bool enabled,
    float width) {
  ImGui::BeginDisabled(!enabled);
  if (selected) {
    ImGui::PushStyleColor(ImGuiCol_Button,
        ImGui::GetStyleColorVec4(ImGuiCol_HeaderActive));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
        ImGui::GetStyleColorVec4(ImGuiCol_HeaderHovered));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,
        ImGui::GetStyleColorVec4(ImGuiCol_HeaderActive));
    ImGui::PushStyleColor(ImGuiCol_Border,
        ImGui::GetStyleColorVec4(ImGuiCol_CheckMark));
  }

  const bool clicked =
      ImGui::Button(label, ImVec2(width, UI::Ui(AutopilotSelectorHeight)));

  if (selected) {
    ImGui::PopStyleColor(4);
  }
  ImGui::EndDisabled();
  return enabled && clicked;
}

void DrawAutopilotSourceSelector(AutopilotViewState &viewState,
    bool baselineAvailable) {
  const AutopilotSelection selection = viewState.GetSelection();
  ImGui::TextDisabled("EDIT AUTOPILOT");
  const float spacing = UI::Ui(AutopilotSelectorSpacing);
  const float segmentWidth =
      std::max((ImGui::GetContentRegionAvail().x - spacing) * 0.5F, 1.0F);
  ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding,
      UI::Ui(AutopilotSelectorRounding));
  ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0F);
  if (DrawAutopilotSegment("PRIMARY",
          selection == AutopilotSelection::Primary,
          true,
          segmentWidth)) {
    viewState.Select(AutopilotSelection::Primary, baselineAvailable);
  }
  ImGui::SameLine(0.0F, spacing);
  if (DrawAutopilotSegment("BASELINE",
          selection == AutopilotSelection::Baseline,
          baselineAvailable,
          segmentWidth)) {
    viewState.Select(AutopilotSelection::Baseline, baselineAvailable);
  }
  ImGui::PopStyleVar(2);
  if (!baselineAvailable
      && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip("Baseline autopilot is not available");
  }
  ImGui::Spacing();
}

const char *GetAutopilotStrategyName(const gnc::IAutopilot &autopilot) {
  if (dynamic_cast<const gnc::MyAutopilot *>(&autopilot) != nullptr) {
    return "MyAutopilot";
  }
  if (dynamic_cast<const gnc::PX4Autopilot *>(&autopilot) != nullptr) {
    return "PX4Autopilot";
  }
  return "Autopilot Strategy";
}

void DrawAutopilotIdentityHeader(AutopilotSelection selection,
    const char *strategyName) {
  const char *role =
      selection == AutopilotSelection::Primary ? "PRIMARY" : "BASELINE";
  const float height = UI::Ui(AutopilotIdentityHeight);
  const float width = std::max(ImGui::GetContentRegionAvail().x, 1.0F);
  const ImVec2 minimum = ImGui::GetCursorScreenPos();
  const ImVec2 maximum{minimum.x + width, minimum.y + height};
  ImGui::Dummy(ImVec2(width, height));

  ImDrawList &drawList = *ImGui::GetWindowDrawList();
  drawList.AddRectFilled(minimum,
      maximum,
      ImGui::GetColorU32(ImGuiCol_FrameBg));
  drawList.AddRect(minimum,
      maximum,
      ImGui::GetColorU32(ImGuiCol_Border),
      UI::Ui(AutopilotSelectorRounding));
  drawList.AddRectFilled(minimum,
      ImVec2(minimum.x + UI::Ui(AutopilotIdentityAccentWidth), maximum.y),
      ImGui::GetColorU32(ImGuiCol_CheckMark),
      UI::Ui(AutopilotSelectorRounding));

  const std::string roleLabel = std::string(role) + "  ·  ";
  const ImVec2 textSize = ImGui::CalcTextSize(roleLabel.c_str());
  const ImVec2 textPosition{minimum.x + UI::Ui(AutopilotIdentityPadding),
      minimum.y + (height - ImGui::GetTextLineHeight()) * 0.5F};
  drawList.PushClipRect(minimum, maximum, true);
  drawList.AddText(textPosition,
      ImGui::GetColorU32(ImGuiCol_CheckMark),
      roleLabel.c_str());
  drawList.AddText(ImVec2(textPosition.x + textSize.x, textPosition.y),
      ImGui::GetColorU32(ImGuiCol_Text),
      strategyName);
  drawList.PopClipRect();
  ImGui::Spacing();
}
} // namespace

GNCWindow::GNCWindow() : Window("GNC", EditorIconAliases::GNC) {}

void GNCWindow::RequestTrim(PendingTrimCommand command) {
  if (pendingTrimCommand_ != PendingTrimCommand::None || trimInProgress_) {
    return;
  }

  pendingTrimCommand_ = command;
}

void GNCWindow::ExecutePendingTrim(gui::GUI &gui) {
  const PendingTrimCommand command = pendingTrimCommand_;
  if (command == PendingTrimCommand::None || trimInProgress_) {
    return;
  }

  pendingTrimCommand_ = PendingTrimCommand::None;
  trimInProgress_ = true;

  auto &simulation = gui.GetPrimarySimulation();
  auto &executionControl = gui.GetSimulationExecutionControl();
  auto &aircraft = simulation.GetAircraft();
  const double simTime = aircraft.GetAircraftState().simulationTimeSec;
  const bool resumeAfterTrim =
      executionControl.GetSimulationExecutionState()
      == application::SimulationExecutionState::Running;
  executionControl.PauseSimulation();

  const char *commandLabel = "None";
  switch (command) {
  case PendingTrimCommand::RunInitialCondition:
    commandLabel = "RunIC";
    break;
  case PendingTrimCommand::CurrentState:
    commandLabel = "CurrentState";
    break;
  case PendingTrimCommand::None:
  default:
    break;
  }

  std::cout << "[GNC] trim request command=" << commandLabel
            << " simTime=" << simTime << '\n';

  auto *flightControlManager =
      simulation.GetComponent<control::FlightControlManager>();
  if (flightControlManager == nullptr) {
    trimInProgress_ = false;
    if (resumeAfterTrim) {
      executionControl.ResumeSimulation();
    }
    return;
  }
  auto &trimService = simulation.GetTrimService();
  bool trimSuccess = false;
  if (command == PendingTrimCommand::RunInitialCondition) {
    trimSuccess = trimService.Compute(aircraft, trimRequest_);
  } else if (command == PendingTrimCommand::CurrentState) {
    trimSuccess = trimService.ComputeCurrentState(aircraft, trimRequest_.mode);
  }

  if (trimSuccess && trimService.ApplyStored(aircraft)) {
    if (const gnc::TrimResult *trimResult = trimService.GetResult()) {
      flightControlManager->SynchronizeWithTrimResult(aircraft, *trimResult);
    }
  }

  trimResultOpen_ = true;
  trimResidualOpen_ = true;
  trimInProgress_ = false;

  if (resumeAfterTrim) {
    executionControl.ResumeSimulation();
  }
}

void GNCWindow::OnRender(gui::GUI &gui) {
  const bool baselineAvailable = IsBaselineAutopilotAvailable(gui);
  autopilotViewState_.EnsureBaselineAvailable(baselineAvailable);

  auto &primarySimulation = gui.GetPrimarySimulation();
  auto &primaryAircraft = primarySimulation.GetAircraft();
  auto *primaryFlightControlManager =
      primarySimulation.GetComponent<control::FlightControlManager>();
  if (primaryFlightControlManager == nullptr) {
    return;
  }
  auto &manualController = primaryFlightControlManager->GetManualController();
  const gnc::TrimResult emptyTrimResult{};
  const gnc::TrimResult *trimResult =
      primarySimulation.GetTrimService().GetResult();
  const bool trimHasResult = trimResult != nullptr;

  // clang-format off
  UI::VerticalLayout()
      [
        +UI::TabGroup("GNC")
            [
              +UI::Tab("Trim")
                  [
                    UI::Custom([this,
                                   emptyTrimResult,
                                   trimResult,
                                   trimHasResult] {
                      TrimPanel::Draw({
                          trimRequest_,
                          trimHasResult ? *trimResult : emptyTrimResult,
                          trimHasResult,
                          trimResultOpen_,
                          trimResidualOpen_,
                          !trimInProgress_
                              && pendingTrimCommand_ == PendingTrimCommand::None,
                          [this] {
                            RequestTrim(PendingTrimCommand::RunInitialCondition);
                          },
                          [this] {
                            RequestTrim(PendingTrimCommand::CurrentState);
                          },
                      });
                    })
                  ]
              + UI::Tab("Autopilot")
                    [
                      UI::Custom([this,
                                     &gui,
                                     baselineAvailable] {
                        DrawAutopilotSourceSelector(
                            autopilotViewState_, baselineAvailable);
                        const AutopilotSelection selection =
                            autopilotViewState_.GetSelection();
                        sim::Simulation &simulation = GetAutopilotSimulation(
                            gui, selection);
                        auto *flightControlManager = simulation.GetComponent<
                            control::FlightControlManager>();
                        if (flightControlManager == nullptr) {
                          DrawAutopilotIdentityHeader(
                              selection, "Unavailable");
                          ImGui::TextDisabled(
                              "Flight control manager is unavailable.");
                          return;
                        }
                        gnc::IAutopilot &autopilotStrategy =
                            flightControlManager->GetAutopilot();
                        DrawAutopilotIdentityHeader(selection,
                            GetAutopilotStrategyName(autopilotStrategy));
                        if (selection == AutopilotSelection::Baseline) {
                          auto *px4Autopilot =
                              dynamic_cast<gnc::PX4Autopilot *>(
                                  &autopilotStrategy);
                          if (px4Autopilot == nullptr) {
                            ImGui::TextDisabled(
                                "Roll Hold settings are not available for "
                                "this baseline strategy.");
                            return;
                          }

                          const auto &px4Diagnostics =
                              px4Autopilot->GetRollHoldDiagnostics();
                          auto &baselineAircraft = simulation.GetAircraft();
                          const auto &baselineProperties =
                              baselineAircraft.GetProperties();
                          BaselineAutopilotPanel::Draw({
                              .state = baselineAutopilotPanelState_,
                              .currentRollDeg =
                                  baselineProperties.Roll().Deg(),
                              .currentRollRateDegPerSec =
                                  baselineProperties.P().DegPerSec(),
                              .currentAileron =
                                  baselineAircraft.GetControls().GetAileron(),
                              .rollHoldActive =
                                  flightControlManager->GetMode()
                                      == control::FlightControlMode::Autopilot
                                  && px4Autopilot->IsRollHoldEnabled(),
                              .captureCurrentRoll = [this,
                                                        &baselineProperties] {
                                baselineAutopilotPanelState_.rollTargetDeg =
                                    baselineProperties.Roll().Deg();
                              },
                              .px4RollAileronCommand =
                                  px4Diagnostics.aileronCommand,
                              .px4RollRateSetpointDegPerSec = math::RadToDeg(
                                  px4Diagnostics.bodyRateSetpointRadPerSec),
                              .px4RollErrorDeg = math::RadToDeg(
                                  px4Diagnostics.rollErrorRad),
                              .px4AirspeedScaling =
                                  px4Diagnostics.airspeedScaling,
                          });
                          return;
                        }

                        auto *myAutopilot = dynamic_cast<gnc::MyAutopilot *>(
                            &autopilotStrategy);
                        if (myAutopilot == nullptr) {
                          ImGui::TextDisabled(
                              "Tuning controls are not available for this "
                              "autopilot strategy yet.");
                          return;
                        }

                        auto &aircraft = simulation.GetAircraft();
                        auto &autopilot = *myAutopilot;
                        AutopilotPanelState &panelState =
                            primaryAutopilotPanelState_;
                        const auto &properties = aircraft.GetProperties();
                        const bool autopilotMode =
                            flightControlManager->GetMode()
                            == control::FlightControlMode::Autopilot;
                        const bool rollHoldEnabled =
                            autopilot.IsRollHoldEnabled();
                        const bool rollDynamicsReady =
                            autopilot.GetRollDynamics().has_value();
                        AutopilotPanel::Draw({
                            .state = panelState,
                            .currentRollDeg = properties.Roll().Deg(),
                            .currentRollRateDegPerSec =
                                properties.P().DegPerSec(),
                            .currentAileron =
                                aircraft.GetControls().GetAileron(),
                            .rollHoldActive =
                                autopilotMode && rollHoldEnabled
                                && rollDynamicsReady,
                            .rollHoldPreparing =
                                autopilotMode && rollHoldEnabled
                                && !rollDynamicsReady,
                            .captureCurrentRoll = [&panelState, &properties] {
                              panelState.rollTargetDeg = properties.Roll().Deg();
                            },
                        });
                      })
                    ]
              + UI::Tab("Flight Control")
                    [
                      UI::Custom([&manualController, &primaryAircraft, this] {
                        ManualControlPanel::Draw(
                            manualController,
                            primaryAircraft,
                            primaryAutopilotPanelState_);
                      })
                    ]
            ]
      ]
      .Render();
  // clang-format on

  sim::Simulation *baselineSimulation = gui.GetBaselineSimulation();
  auto *baselineManager =
      baselineSimulation != nullptr
          ? baselineSimulation->GetComponent<control::FlightControlManager>()
          : nullptr;
  auto *baselineAutopilot =
      baselineManager != nullptr
          ? dynamic_cast<gnc::PX4Autopilot *>(&baselineManager->GetAutopilot())
          : nullptr;
  const bool scenarioActive =
      gui.GetSimulationExecutionControl().GetScenarioExecutionStatus()
          .has_value();
  if (!scenarioActive) {
    if (auto *autopilot = dynamic_cast<gnc::MyAutopilot *>(
            &primaryFlightControlManager->GetAutopilot())) {
      autopilot->SetRollHoldEnabled(primaryAutopilotPanelState_.rollHold);
      autopilot->SetRollHoldSettings(
          MakeRollHoldSettings(primaryAutopilotPanelState_));
      primaryFlightControlManager->SetMode(
          HasAnyAutopilotControlEnabled(primaryAutopilotPanelState_)
              ? control::FlightControlMode::Autopilot
              : control::FlightControlMode::Manual);
    }
  }

  if (!scenarioActive && baselineAutopilot != nullptr) {
    ApplyBaselinePx4RollSettings(*baselineAutopilot,
        baselineAutopilotPanelState_);
    baselineAutopilot->SetTargetRollRad(
        math::DegToRad(baselineAutopilotPanelState_.rollTargetDeg));
    baselineAutopilot->SetRollHoldEnabled(
        baselineAutopilotPanelState_.rollHold);
    baselineManager->SetMode(baselineAutopilotPanelState_.rollHold
                                 ? control::FlightControlMode::Autopilot
                                 : control::FlightControlMode::Manual);
  }
  ExecutePendingTrim(gui);
}
} // namespace gui
