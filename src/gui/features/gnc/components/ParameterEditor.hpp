#pragma once

#include "flightui/FlightUI.hpp"
#include "sim/gnc/parameters/Parameter.hpp"

#include <string>
#include <string_view>

namespace gui {

template <typename Parameter, typename OnChanged>
ui::PropertyRowBuilder RenderGncParameterRow(
    const gnc::ParameterMetadata<Parameter> &metadata,
    std::string_view editorSuffix, double value, OnChanged onChanged) {
  constexpr float AdaptivePropertyInputWidth = 0.0F;
  const std::string editorId =
      std::string(metadata.id) + std::string(editorSuffix);
  const std::string_view displayUnit = gnc::GetUnitSymbol(metadata.displayUnit);
  const std::string tooltip = displayUnit.empty()
                                  ? std::string(metadata.displayName)
                                  : std::string(metadata.displayName) + " ("
                                        + std::string(displayUnit) + ")";
  const double displayMinimum =
      gnc::ToParameterDisplayValue(metadata, metadata.minimum);
  const double displayMaximum =
      gnc::ToParameterDisplayValue(metadata, metadata.maximum);
  const double displayValue = gnc::ToParameterDisplayValue(metadata, value);
  const double displayIncrement = gnc::GetParameterDisplayIncrement(metadata);

  return ui::PropertyRow(std::string(metadata.id))
      .Tooltip(tooltip)[ui::ScalarEditor(editorId, displayValue)
              .Range(displayMinimum, displayMaximum)
              .ShowSlider(false)
              .ShowStepper()
              .Step(displayIncrement)
              .FastStep(displayIncrement * 10.0)
              .Format("%.3f")
              .InputWidth(AdaptivePropertyInputWidth)
              .Tooltip(tooltip)
              .OnChanged([metadata, onChanged](double displayValue) {
                onChanged(
                    gnc::FromParameterDisplayValue(metadata, displayValue));
              })];
}
} // namespace gui
