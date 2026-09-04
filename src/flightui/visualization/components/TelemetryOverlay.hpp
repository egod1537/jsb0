#pragma once

#include "flightui/visualization/core/Component.hpp"
#include "sim/AircraftState.hpp"

#include <string>

namespace viz {
std::string FormatTelemetryFlightState(const sim::AircraftState &state);
std::string FormatTelemetryAttitude(const sim::AircraftState &state);

class TelemetryOverlay final : public Component {
public:
  void Render(RenderContext &context) const override;
};
} // namespace viz
