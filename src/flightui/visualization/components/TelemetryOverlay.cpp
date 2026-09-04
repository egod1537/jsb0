#include "flightui/visualization/components/TelemetryOverlay.hpp"

#include "common/math/Math.hpp"
#include "flightui/visualization/render/LineCanvas.hpp"
#include "flightui/core/UIScale.hpp"

#include <cstdio>

namespace viz {
std::string FormatTelemetryFlightState(const sim::AircraftState &state) {
  char line[160]{};
  std::snprintf(line,
      sizeof(line),
      "Alt AGL %.1f m  Course %.1f deg  CAS %.1f m/s  TAS %.1f m/s",
      state.altitudeAglM,
      math::RadToDeg(state.courseRad),
      state.calibratedAirspeedMps,
      state.trueAirspeedMps);
  return line;
}

std::string FormatTelemetryAttitude(const sim::AircraftState &state) {
  char line[160]{};
  std::snprintf(line,
      sizeof(line),
      "Roll %.1f deg  Pitch %.1f deg  Heading %.1f deg",
      math::RadToDeg(state.rollRad),
      math::RadToDeg(state.pitchRad),
      math::RadToDeg(state.headingRad));
  return line;
}

void TelemetryOverlay::Render(RenderContext &context) const {
  if (!context.snapshot.viewOptions.showTelemetry) {
    return;
  }

  const AircraftSnapshot &aircraft = context.snapshot.aircraft;
  const auto &aircraftState = aircraft.state;
  const auto &controlInput = aircraft.controlInput;
  const char *viewMode = context.snapshot.viewMode == ViewMode::ThirdPerson
                             ? "Third Person"
                             : "Orbit";
  const ImVec2 min = context.canvas.GetMin();
  ImDrawList &drawList = context.canvas.GetDrawList();

  char line[160]{};
  std::snprintf(line,
      sizeof(line),
      "t %.2f  View %s",
      aircraftState.simulationTimeSec,
      viewMode);
  drawList.AddText(
      ImVec2(min.x + ui::Ui(10.0F), min.y + ui::Ui(10.0F)),
      IM_COL32(232, 238, 246, 255),
      line);

  const std::string flightState = FormatTelemetryFlightState(aircraftState);
  drawList.AddText(
      ImVec2(min.x + ui::Ui(10.0F), min.y + ui::Ui(30.0F)),
      IM_COL32(232, 238, 246, 255),
      flightState.c_str());

  const std::string attitude = FormatTelemetryAttitude(aircraftState);
  drawList.AddText(
      ImVec2(min.x + ui::Ui(10.0F), min.y + ui::Ui(50.0F)),
      IM_COL32(232, 238, 246, 255),
      attitude.c_str());

  std::snprintf(line,
      sizeof(line),
      "Ail %.2f  Ele %.2f  Rud %.2f  Thr %.2f  Trim %.2f",
      controlInput.aileron,
      controlInput.elevator,
      controlInput.rudder,
      controlInput.throttle,
      aircraft.pitchTrim);
  drawList.AddText(
      ImVec2(min.x + ui::Ui(10.0F), min.y + ui::Ui(70.0F)),
      IM_COL32(178, 189, 202, 255),
      line);
}
} // namespace viz
