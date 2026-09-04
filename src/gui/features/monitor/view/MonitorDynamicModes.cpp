#include "gui/features/monitor/MonitorView.hpp"

#include "sim/linearization/DynamicModeContracts.hpp"

#include "flightui/FlightUI.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>

namespace gui {

namespace {
constexpr std::size_t MaximumDisplayedModeStates = 6;
constexpr double MinimumDisplayedParticipation = 0.05;

const gnc::DynamicModeSnapshot *FindLatestDynamicModeAtOrBefore(
    std::span<const gnc::DynamicModeSnapshot> history, double timeSec) {
  const auto snapshot = std::upper_bound(history.begin(),
      history.end(),
      timeSec,
      [](double time, const gnc::DynamicModeSnapshot &candidate) {
        return time < candidate.simulationTimeSec;
      });
  return snapshot == history.begin() ? nullptr : &*std::prev(snapshot);
}
} // namespace

void MonitorView::DrawDynamicModes(
    const MonitorDynamicModeInput &dynamicModes) {
  if (!dynamicModes.available) {
    ImGui::TextDisabled(
        "Dynamic mode analysis is not available for this autopilot.");
    return;
  }

  bool automaticUpdates = dynamicModes.automaticUpdatesEnabled;
  if (ImGui::Checkbox("Automatic linearization", &automaticUpdates)) {
    events_.Emit(MonitorAutomaticLinearizationChanged{automaticUpdates});
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Run asynchronous aircraft linearization every 5 seconds");
  }
  ImGui::SameLine();

  const bool updateInProgress = dynamicModes.updateInProgress;
  const std::string_view updateError = dynamicModes.errorMessage;

  if (!automaticUpdates) {
    ImGui::TextDisabled(updateInProgress ? "Off (current worker is finishing)"
                                         : "Off (latest result retained)");
  } else if (updateInProgress) {
    ImGui::TextDisabled("Updating linearization asynchronously...");
  } else if (!updateError.empty()) {
    ImGui::TextColored(ui::GetDarkEditorSemanticColor(ui::SemanticColor::Error),
        "Latest linearization failed: %.*s",
        static_cast<int>(updateError.size()),
        updateError.data());
  }

  if (!selectedTimeInitialized_) {
    ImGui::Separator();
    ImGui::TextDisabled("Waiting for a Monitor timeline time.");
    return;
  }
  const gnc::DynamicModeSnapshot *snapshot =
      FindLatestDynamicModeAtOrBefore(dynamicModes.history, selectedTimeSec_);
  if (snapshot == nullptr) {
    selectedDynamicModeIndex_.reset();
    selectedDynamicModeSnapshotTimeSec_.reset();
    ImGui::Separator();
    ImGui::Text("Timeline time: %.3f s", selectedTimeSec_);
    ImGui::TextDisabled("No linearization available at this time.");
    return;
  }

  const gnc::DynamicModeAnalysis *analysis = &snapshot->analysis;
  if (!analysis->valid) {
    ImGui::Separator();
    ImGui::TextColored(ui::GetDarkEditorSemanticColor(ui::SemanticColor::Error),
        "Dynamic-mode analysis is unavailable: %s",
        analysis->errorMessage.c_str());
    return;
  }

  const double ageSec =
      std::max(0.0, selectedTimeSec_ - snapshot->simulationTimeSec);
  ImGui::TextDisabled("Timeline %.3f s  |  Linearization %.3f s  |  Age %.3f s",
      selectedTimeSec_,
      snapshot->simulationTimeSec,
      ageSec);
  ImGui::TextDisabled("Linearization: Valid  |  Full A: %zu modes",
      analysis->modes.size());
  ImGui::Separator();

  if (analysis->modes.empty()) {
    ImGui::TextDisabled("No dynamic modes were detected.");
    return;
  }
  if (!selectedDynamicModeSnapshotTimeSec_
      || *selectedDynamicModeSnapshotTimeSec_ != snapshot->simulationTimeSec) {
    selectedDynamicModeSnapshotTimeSec_ = snapshot->simulationTimeSec;
    selectedDynamicModeIndex_ = 0;
  } else if (!selectedDynamicModeIndex_
             || *selectedDynamicModeIndex_ >= analysis->modes.size()) {
    selectedDynamicModeIndex_ = 0;
  }

  constexpr ImGuiTableFlags ModeTableFlags =
      ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
      | ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp;
  if (ImGui::BeginTable("DynamicModeTable", 6, ModeTableFlags)) {
    ImGui::TableSetupColumn("Mode", ImGuiTableColumnFlags_WidthStretch, 1.3F);
    ImGui::TableSetupColumn("Eigenvalue",
        ImGuiTableColumnFlags_WidthStretch,
        1.5F);
    ImGui::TableSetupColumn("wn (rad/s)",
        ImGuiTableColumnFlags_WidthStretch,
        0.9F);
    ImGui::TableSetupColumn("zeta", ImGuiTableColumnFlags_WidthStretch, 0.7F);
    ImGui::TableSetupColumn("Period", ImGuiTableColumnFlags_WidthStretch, 0.8F);
    ImGui::TableSetupColumn("Stability",
        ImGuiTableColumnFlags_WidthStretch,
        0.9F);
    ImGui::TableHeadersRow();

    for (std::size_t modeIndex = 0; modeIndex < analysis->modes.size();
        ++modeIndex) {
      const gnc::DynamicMode &mode = analysis->modes[modeIndex];
      ImGui::PushID(static_cast<int>(modeIndex));
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      const std::string_view modeName = gnc::ToString(mode.classification);
      if (ImGui::Selectable(modeName.data(),
              selectedDynamicModeIndex_ == modeIndex,
              ImGuiSelectableFlags_SpanAllColumns)) {
        selectedDynamicModeIndex_ = modeIndex;
      }

      ImGui::TableSetColumnIndex(1);
      if (std::abs(mode.eigenvalue.imag()) > 0.0) {
        ImGui::Text("%.3f \xC2\xB1 %.3fi",
            mode.eigenvalue.real(),
            std::abs(mode.eigenvalue.imag()));
      } else {
        ImGui::Text("%.3f", mode.eigenvalue.real());
      }
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.3f", mode.naturalFrequencyRadPerSec);
      ImGui::TableSetColumnIndex(3);
      if (mode.dampingRatio) {
        ImGui::Text("%.3f", *mode.dampingRatio);
      } else {
        ImGui::TextDisabled("--");
      }
      ImGui::TableSetColumnIndex(4);
      if (mode.periodSec) {
        ImGui::Text("%.3f s", *mode.periodSec);
      } else {
        ImGui::TextDisabled("--");
      }
      ImGui::TableSetColumnIndex(5);
      const ui::SemanticColor stabilityColor =
          mode.stability == gnc::DynamicModeStability::Stable
              ? ui::SemanticColor::Success
          : mode.stability == gnc::DynamicModeStability::Unstable
              ? ui::SemanticColor::Error
              : ui::SemanticColor::Warning;
      const std::string_view stability = gnc::ToString(mode.stability);
      ImGui::TextColored(ui::GetDarkEditorSemanticColor(stabilityColor),
          "%.*s",
          static_cast<int>(stability.size()),
          stability.data());
      ImGui::PopID();
    }
    ImGui::EndTable();
  }

  const gnc::DynamicMode &selectedMode =
      analysis->modes[*selectedDynamicModeIndex_];
  const std::string_view selectedModeName =
      gnc::ToString(selectedMode.classification);
  ImGui::Spacing();
  ImGui::SeparatorText("Dominant States");
  ImGui::Text("%.*s",
      static_cast<int>(selectedModeName.size()),
      selectedModeName.data());

  constexpr ImGuiTableFlags ParticipationTableFlags =
      ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersInnerH
      | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
  if (ImGui::BeginTable("DynamicModeParticipation",
          2,
          ParticipationTableFlags)) {
    ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthStretch, 1.0F);
    ImGui::TableSetupColumn("Normalized participation",
        ImGuiTableColumnFlags_WidthStretch,
        2.0F);
    ImGui::TableHeadersRow();

    std::size_t displayedCount = 0;
    for (const gnc::DynamicModeStateParticipation &state :
        selectedMode.stateParticipations) {
      if (displayedCount >= MaximumDisplayedModeStates
          || (displayedCount > 0
              && state.normalizedMagnitude < MinimumDisplayedParticipation)) {
        break;
      }
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextUnformatted(state.stateName.c_str());
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.2f", state.normalizedMagnitude);
      ++displayedCount;
    }
    ImGui::EndTable();
  }
}
} // namespace gui