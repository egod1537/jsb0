#include "gui/features/monitor/MonitorView.hpp"

#include "gui/features/monitor/catalog/MonitorPlotPresetCatalog.hpp"

#include "flightui/FlightUI.hpp"

#include <imgui.h>

namespace gui {
namespace UI = FlightUI;

namespace {
enum class PresetSelectionAction {
  NoChange,
  SelectAll,
  SelectNone,
  Reset,
};

PresetSelectionAction DrawPresetSelectionHeader() {
  PresetSelectionAction action = PresetSelectionAction::NoChange;
  const UI::UIElement toolbar =
      UI::Toolbar()
          .Id("PresetSelection")
          .Compact()
          .Height(26.0F)
          .Left(UI::Text("Selection"))
          .Right(UI::HorizontalLayout().Spacing(
              4.0F)[+UI::Button("All").OnAction([&action] {
            action = PresetSelectionAction::SelectAll;
          }) + UI::Button("None").OnAction([&action] {
            action = PresetSelectionAction::SelectNone;
          }) + UI::Button("Reset").OnAction([&action] {
            action = PresetSelectionAction::Reset;
          })]);
  toolbar.Render();
  return action;
}
} // namespace

void MonitorView::DrawPresetPaneHeader() {
  if (ImGui::Button("<##CloseMonitorPresets")) {
    presetPaneOpen_ = false;
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Collapse Monitor Presets");
  }
  ImGui::SameLine();
  ImGui::TextUnformatted("Monitor Presets");
}

void MonitorView::DrawPresetPanel() {
  const PresetSelectionAction selectionAction = DrawPresetSelectionHeader();
  if (selectionAction == PresetSelectionAction::SelectAll) {
    activePresetMask_ = GetAllMonitorPresetMask();
  } else if (selectionAction == PresetSelectionAction::SelectNone) {
    activePresetMask_ = 0;
  } else if (selectionAction == PresetSelectionAction::Reset) {
    InitializePresetWorkspace();
  }

  constexpr ImGuiTreeNodeFlags CategoryFlags =
      ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_OpenOnArrow
      | ImGuiTreeNodeFlags_SpanAvailWidth;

  const std::span categories = GetMonitorPresetCategoryDefinitions();
  const std::span presets = GetMonitorPresetDefinitions();
  for (const MonitorPresetCategoryDefinition &category : categories) {
    ImGui::PushID(static_cast<int>(category.category));
    const bool isOpen = ImGui::TreeNodeEx(category.name.data(), CategoryFlags);
    if (isOpen) {
      for (std::size_t presetIndex = 0; presetIndex < presets.size();
          ++presetIndex) {
        const MonitorPresetDefinition &preset = presets[presetIndex];
        if (preset.category != category.category) {
          continue;
        }

        bool active = IsPresetActive(presetIndex);
        ImGui::PushID(static_cast<int>(presetIndex));
        if (ImGui::Checkbox(preset.name.data(), &active)) {
          SetPresetActive(presetIndex, active);
        }
        ImGui::PopID();
      }
      ImGui::TreePop();
    }
    ImGui::PopID();
  }
}

bool MonitorView::IsPlotVisible(const MonitorPlot &plot) const {
  return plot.custom || IsPlotVisibleByPreset(plot);
}

bool MonitorView::IsPlotVisibleByPreset(const MonitorPlot &plot) const {
  return gui::IsMonitorPlotVisibleByPreset(plot.telemetryGroupPath,
      activePresetMask_);
}

bool MonitorView::IsPresetActive(std::size_t presetIndex) const {
  return gui::IsMonitorPresetActive(activePresetMask_, presetIndex);
}

void MonitorView::SetPresetActive(std::size_t presetIndex, bool active) {
  const std::span presets = GetMonitorPresetDefinitions();
  if (presetIndex >= presets.size()) {
    return;
  }
  const std::uint32_t presetBit = GetPresetBit(presets[presetIndex].preset);
  if (active) {
    activePresetMask_ |= presetBit;
  } else {
    activePresetMask_ &= ~presetBit;
  }
}
} // namespace gui