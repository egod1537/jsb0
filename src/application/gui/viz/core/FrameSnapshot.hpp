#pragma once

#include "application/gui/viz/core/Math.hpp"
#include "application/sim/AircraftState.hpp"
#include "application/sim/control/ControlInput.hpp"

namespace viz {
enum class ViewMode {
  Orbit,
  ThirdPerson,
};

struct ViewOptions {
  bool showGroundGrid = true;
  bool showTelemetry = true;
  bool showMinimap = true;
};

struct AircraftSnapshot {
  sim::AircraftState state{};
  control::ControlInput controlInput{};
  double pitchTrim = 0.0;
  Vec3 position{};
  float visualAltitude = 0.0F;
  bool available = false;
};

struct FrameSnapshot {
  AircraftSnapshot aircraft{};
  AircraftSnapshot shadowAircraft{};
  ViewMode viewMode = ViewMode::Orbit;
  ViewOptions viewOptions{};
  Vec3 groundScroll{};
  bool shadowEnabled = false;
};
} // namespace viz
