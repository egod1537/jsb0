#include "gui/features/monitor/MonitorView.hpp"

#include "sim/telemetry/TelemetryMetadata.hpp"

#include "flightui/FlightUI.hpp"

#include <imgui.h>

#include <algorithm>
#include <map>
#include <utility>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float PlotDialogWidth = 720.0F;
constexpr float PlotDialogHeight = 690.0F;
constexpr float PlotDialogMinimumWidth = 520.0F;
constexpr float PlotDialogMinimumHeight = 460.0F;
constexpr float PlotDialogViewportMargin = 32.0F;
constexpr float SignalListHeight = 250.0F;
} // namespace

std::vector<MonitorSignalDescriptor> MonitorView::BuildSignalCatalog(
    const TelemetrySources &sources) const {
  std::map<std::string, telemetry::TelemetrySignalMetadata, std::less<>>
      metadataByPath;
  const auto appendSource = [&metadataByPath](
                                const telemetry::TelemetrySnapshot *snapshot) {
    if (snapshot == nullptr) {
      return;
    }
    for (const telemetry::TelemetrySignalMetadata &metadata :
        snapshot->metadata) {
      metadataByPath.insert_or_assign(metadata.path, metadata);
    }
    for (const telemetry::TelemetrySeries &series : snapshot->series) {
      metadataByPath.try_emplace(series.path,
          telemetry::ResolveTelemetrySignalMetadata(series.path));
    }
  };
  appendSource(sources.primary.get());
  appendSource(sources.baseline.get());
  for (const MonitorPlot &plot : plots_) {
    if (plot.custom) {
      continue;
    }
    for (const std::string &channel : plot.channels) {
      metadataByPath.try_emplace(channel,
          telemetry::ResolveTelemetrySignalMetadata(channel));
    }
  }
  std::vector<telemetry::TelemetrySignalMetadata> metadata;
  metadata.reserve(metadataByPath.size());
  for (auto &entry : metadataByPath) {
    metadata.push_back(std::move(entry.second));
  }
  return BuildMonitorSignalCatalog(metadata, plots_);
}

void MonitorView::RequestAddPlot(std::optional<std::size_t> slotIndex) {
  if (slotIndex
      && (*slotIndex >= GetAvailablePlotSlotCount()
          || renderState_.customPlotSlots[*slotIndex].has_value())) {
    slotIndex.reset();
  }
  plotDialog_.BeginAdd(slotIndex);
  noEmptySlotMessage_ = !slotIndex.has_value();
}

void MonitorView::RequestEditPlot(MonitorPlot &plot) {
  plotDialog_.BeginEdit(plot);
}

void MonitorView::CommitPlotDialog(
    std::span<const MonitorSignalDescriptor> signalCatalog) {
  const std::string title = ResolveMonitorPlotTitle(plotDialog_.title.data(),
      plotDialog_.selectedSignalIds,
      signalCatalog);
  const MonitorSignalDescriptor *firstSignal =
      plotDialog_.selectedSignalIds.empty()
          ? nullptr
          : FindMonitorSignal(signalCatalog, plotDialog_.selectedSignalIds[0]);
  const std::string unit = firstSignal == nullptr || firstSignal->unit.empty()
                               ? "Value"
                               : firstSignal->unit;

  if (plotDialog_.editingPlotId) {
    if (MonitorPlot *plot = FindPlot(*plotDialog_.editingPlotId)) {
      plot->title = title;
      plot->channels = plotDialog_.selectedSignalIds;
      plot->yAxisLabel = unit;
      plot->showLegend = plotDialog_.showLegend;
      plot->manualYAxis = plotDialog_.manualYAxis;
      plot->yAxisMinimum = plotDialog_.yAxisMinimum;
      plot->yAxisMaximum = plotDialog_.yAxisMaximum;
      plot->templateId = plotDialog_.selectedTemplateId;
      plot->telemetryGroupPath = plotDialog_.selectedTemplateId;
      plot->hiddenSeries.clear();
    }
    return;
  }

  if (!plotDialog_.targetSlot) {
    return;
  }
  MonitorPlot plot{.title = title,
      .channels = plotDialog_.selectedSignalIds,
      .telemetryGroupPath = plotDialog_.selectedTemplateId,
      .yAxisLabel = unit,
      .custom = true,
      .showLegend = plotDialog_.showLegend,
      .manualYAxis = plotDialog_.manualYAxis,
      .yAxisMinimum = plotDialog_.yAxisMinimum,
      .yAxisMaximum = plotDialog_.yAxisMaximum,
      .templateId = plotDialog_.selectedTemplateId};
  AddMonitorPlotToSlot(renderState_, std::move(plot), *plotDialog_.targetSlot);
}

void MonitorView::DrawPlotConfigurationDialog(const TelemetrySources &sources) {
  const char *popupId = plotDialog_.editingPlotId ? "Edit Plot" : "Add Plot";
  if (plotDialog_.openRequested) {
    ImGui::OpenPopup(popupId);
    plotDialog_.openRequested = false;
    plotDialog_.focusSearch = true;
  }

  const ImVec2 workSize = ImGui::GetMainViewport()->WorkSize;
  const float maximumWidth =
      std::max(1.0F, workSize.x - UI::Ui(PlotDialogViewportMargin));
  const float maximumHeight =
      std::max(1.0F, workSize.y - UI::Ui(PlotDialogViewportMargin));
  const ImVec2 minimumSize(
      std::min(UI::Ui(PlotDialogMinimumWidth), maximumWidth),
      std::min(UI::Ui(PlotDialogMinimumHeight), maximumHeight));
  const ImVec2 maximumSize(std::max(minimumSize.x, maximumWidth),
      std::max(minimumSize.y, maximumHeight));
  ImGui::SetNextWindowSizeConstraints(minimumSize, maximumSize);
  ImGui::SetNextWindowSize(
      ImVec2(std::min(UI::Ui(PlotDialogWidth), maximumWidth),
          std::min(UI::Ui(PlotDialogHeight), maximumHeight)),
      ImGuiCond_Appearing);

  bool dialogVisible = true;
  if (!ImGui::BeginPopupModal(popupId,
          &dialogVisible,
          ImGuiWindowFlags_NoSavedSettings)) {
    if (!dialogVisible || !ImGui::IsPopupOpen(popupId)) {
      plotDialog_.Close();
    }
    return;
  }

  const std::vector<MonitorSignalDescriptor> signalCatalog =
      BuildSignalCatalog(sources);

  ImGui::TextUnformatted("Plot title");
  ImGui::SetNextItemWidth(-1.0F);
  ImGui::InputText("##PlotTitle",
      plotDialog_.title.data(),
      plotDialog_.title.size());
  ImGui::TextDisabled("Leave blank to use the first signal name.");
  ImGui::Spacing();

  ImGui::SeparatorText("Signals");
  ImGui::TextUnformatted("Add from template");
  const char *templatePreview = "Choose a plot template...";
  if (!plotDialog_.selectedTemplateId.empty()) {
    const auto selectedTemplate =
        std::find_if(plots_.begin(), plots_.end(), [this](const auto &plot) {
          return !plot.custom
                 && plot.telemetryGroupPath == plotDialog_.selectedTemplateId;
        });
    if (selectedTemplate != plots_.end()) {
      templatePreview = selectedTemplate->title.c_str();
    }
  }
  ImGui::SetNextItemWidth(-1.0F);
  if (ImGui::BeginCombo("##PlotTemplate", templatePreview)) {
    for (const MonitorPlot &plot : plots_) {
      if (plot.custom || plot.telemetryGroupPath.empty()) {
        continue;
      }
      const bool selected =
          plotDialog_.selectedTemplateId == plot.telemetryGroupPath;
      ImGui::PushID(static_cast<int>(plot.id));
      if (ImGui::Selectable(plot.title.c_str(), selected)) {
        plotDialog_.selectedTemplateId = plot.telemetryGroupPath;
        plotDialog_.selectedSignalIds = plot.channels;
        plotDialog_.validationMessage.clear();
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
      ImGui::PopID();
    }
    ImGui::EndCombo();
  }

  ImGui::SetNextItemWidth(-1.0F);
  if (plotDialog_.focusSearch) {
    ImGui::SetKeyboardFocusHere();
    plotDialog_.focusSearch = false;
  }
  ImGui::InputTextWithHint("##SignalSearch",
      "Search names, ids, symbols, or units",
      plotDialog_.search.data(),
      plotDialog_.search.size());

  const std::vector<const MonitorSignalDescriptor *> filteredSignals =
      FilterMonitorSignalCatalog(signalCatalog, plotDialog_.search.data());
  const float signalListHeight =
      std::clamp(ImGui::GetContentRegionAvail().y - UI::Ui(235.0F),
          UI::Ui(90.0F),
          UI::Ui(SignalListHeight));
  if (ImGui::BeginChild("SignalList",
          ImVec2(0.0F, signalListHeight),
          true,
          ImGuiWindowFlags_AlwaysVerticalScrollbar)) {
    if (filteredSignals.empty()) {
      ImGui::TextDisabled(signalCatalog.empty()
                              ? "Waiting for telemetry signals."
                              : "No matching signals.");
    } else {
      constexpr ImGuiTableFlags SignalTableFlags =
          ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg
          | ImGuiTableFlags_NoSavedSettings;
      if (ImGui::BeginTable("SignalTable", 4, SignalTableFlags)) {
        ImGui::TableSetupColumn("Signal",
            ImGuiTableColumnFlags_WidthStretch,
            1.5F);
        ImGui::TableSetupColumn("Symbol",
            ImGuiTableColumnFlags_WidthStretch,
            0.45F);
        ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthStretch, 1.6F);
        ImGui::TableSetupColumn("Unit",
            ImGuiTableColumnFlags_WidthStretch,
            0.55F);
        ImGui::TableHeadersRow();

        std::string currentGroup;
        std::string currentSubgroup;
        for (const MonitorSignalDescriptor *signal : filteredSignals) {
          if (signal->group != currentGroup) {
            currentGroup = signal->group;
            currentSubgroup.clear();
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("%s", currentGroup.c_str());
          }
          if (signal->subgroup != currentSubgroup) {
            currentSubgroup = signal->subgroup;
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Indent();
            ImGui::TextDisabled("%s", currentSubgroup.c_str());
            ImGui::Unindent();
          }

          const auto selected = std::find(plotDialog_.selectedSignalIds.begin(),
              plotDialog_.selectedSignalIds.end(),
              signal->id);
          bool enabled = selected != plotDialog_.selectedSignalIds.end();
          const bool incompatible = !enabled
                                    && !CanAddMonitorSignal(signalCatalog,
                                        plotDialog_.selectedSignalIds,
                                        signal->id);

          ImGui::TableNextRow();
          ImGui::TableSetColumnIndex(0);
          ImGui::PushID(signal->id.c_str());
          ImGui::BeginDisabled(incompatible);
          if (ImGui::Checkbox("##Selected", &enabled)) {
            plotDialog_.selectedTemplateId.clear();
            if (enabled) {
              plotDialog_.selectedSignalIds.push_back(signal->id);
            } else {
              plotDialog_.selectedSignalIds.erase(selected);
            }
            plotDialog_.validationMessage.clear();
          }
          ImGui::SameLine();
          ImGui::TextUnformatted(signal->name.c_str());
          ImGui::EndDisabled();
          if (incompatible
              && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            ImGui::SetTooltip("Signals with different units cannot be plotted "
                              "on the same Y axis.");
          }
          ImGui::TableSetColumnIndex(1);
          ImGui::TextDisabled("%s", signal->symbol.c_str());
          ImGui::TableSetColumnIndex(2);
          ImGui::TextUnformatted(signal->id.c_str());
          if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("%s", signal->id.c_str());
          }
          ImGui::TableSetColumnIndex(3);
          ImGui::TextDisabled("%s", signal->unit.c_str());
          ImGui::PopID();
        }
        ImGui::EndTable();
      }
    }
  }
  ImGui::EndChild();
  ImGui::TextDisabled("%zu selected", plotDialog_.selectedSignalIds.size());

  ImGui::Spacing();
  ImGui::SeparatorText("Y axis");
  if (ImGui::RadioButton("Auto", !plotDialog_.manualYAxis)) {
    plotDialog_.manualYAxis = false;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Manual", plotDialog_.manualYAxis)) {
    plotDialog_.manualYAxis = true;
  }

  ImGui::BeginDisabled(!plotDialog_.manualYAxis);
  if (ImGui::BeginTable("YAxisLimits", 2, ImGuiTableFlags_SizingStretchSame)) {
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("Minimum");
    ImGui::SetNextItemWidth(-1.0F);
    ImGui::InputDouble("##Minimum",
        &plotDialog_.yAxisMinimum,
        0.0,
        0.0,
        "%.6g");
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("Maximum");
    ImGui::SetNextItemWidth(-1.0F);
    ImGui::InputDouble("##Maximum",
        &plotDialog_.yAxisMaximum,
        0.0,
        0.0,
        "%.6g");
    ImGui::EndTable();
  }
  ImGui::EndDisabled();

  const bool validYAxis = !plotDialog_.manualYAxis
                          || IsValidMonitorManualYAxis(plotDialog_.yAxisMinimum,
                              plotDialog_.yAxisMaximum);
  const bool targetAvailable =
      plotDialog_.editingPlotId.has_value()
      || (plotDialog_.targetSlot.has_value()
          && *plotDialog_.targetSlot < GetAvailablePlotSlotCount()
          && !renderState_.customPlotSlots[*plotDialog_.targetSlot]
              .has_value());
  if (!validYAxis) {
    ImGui::TextColored(UI::GetDarkEditorSemanticColor(UI::SemanticColor::Error),
        "Minimum and maximum must be finite numbers, and minimum must be less "
        "than maximum.");
  } else if (!plotDialog_.validationMessage.empty()) {
    ImGui::TextColored(UI::GetDarkEditorSemanticColor(UI::SemanticColor::Error),
        "%s",
        plotDialog_.validationMessage.c_str());
  }
  if (!targetAvailable) {
    ImGui::TextColored(
        UI::GetDarkEditorSemanticColor(UI::SemanticColor::Warning),
        "No empty plot slots. Change the layout or remove a plot.");
  }

  ImGui::Checkbox("Show legend", &plotDialog_.showLegend);
  ImGui::Separator();

  const bool canSubmit =
      !plotDialog_.selectedSignalIds.empty() && validYAxis && targetAvailable;
  const char *submitLabel = plotDialog_.editingPlotId ? "Save" : "Add Plot";
  const float footerWidth = ImGui::CalcTextSize("Cancel").x
                            + ImGui::CalcTextSize(submitLabel).x
                            + ImGui::GetStyle().FramePadding.x * 4.0F
                            + ImGui::GetStyle().ItemSpacing.x;
  ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(),
      ImGui::GetWindowContentRegionMax().x - footerWidth));
  if (ImGui::Button("Cancel")) {
    ImGui::CloseCurrentPopup();
    dialogVisible = false;
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(!canSubmit);
  if (ImGui::Button(submitLabel)) {
    CommitPlotDialog(signalCatalog);
    ImGui::CloseCurrentPopup();
    dialogVisible = false;
  }
  ImGui::EndDisabled();

  ImGui::EndPopup();
  if (!dialogVisible) {
    plotDialog_.Close();
  }
}
} // namespace gui