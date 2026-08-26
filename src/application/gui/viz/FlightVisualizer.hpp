#pragma once

#include "application/gui/viz/core/FrameSnapshot.hpp"
#include "application/gui/viz/core/FlightPathHistory.hpp"
#include "application/gui/viz/core/Scene.hpp"

#include <optional>

struct ImVec2;

namespace gui {
struct EditorIconHandle;
}

namespace sim {
class Aircraft;
class Simulation;
} // namespace sim

namespace viz {
class CameraComponent;

class FlightVisualizer {
public:
  // Lifetime
  explicit FlightVisualizer(const sim::Simulation *mainSimulation = nullptr,
      const sim::Simulation *shadowSimulation = nullptr);
  ~FlightVisualizer();

  // Fixed non-owning simulation sources
  const sim::Simulation *GetMainSimulation() const { return mainSimulation_; }
  const sim::Simulation *GetShadowSimulation() const {
    return shadowSimulation_;
  }

  // Frame update
  bool Tick();
  const FrameSnapshot &GetFrameSnapshot() const { return snapshot_; }

  // Visualization state
  ViewMode GetViewMode() const { return viewMode_; }
  void SetViewMode(ViewMode mode);
  bool IsShadowEnabled() const { return shadowEnabled_; }
  void SetShadowEnabled(bool enabled);

  // Rendering
  void RenderScene(const gui::EditorIconHandle &shadowIcon,
      const gui::EditorIconHandle &viewOptionsIcon,
      const gui::EditorIconHandle &cameraViewIcon, const char *shadowTooltip,
      const char *unavailableMessage = "Simulation unavailable.");

private:
  // Scene setup and interaction
  void BuildScene();
  void HandleInput();
  void RenderToolbar(const gui::EditorIconHandle &shadowIcon,
      const gui::EditorIconHandle &viewOptionsIcon,
      const gui::EditorIconHandle &cameraViewIcon, const char *shadowTooltip);
  void RenderViewOptionsPopup();
  void RenderMinimap(ImVec2 min, ImVec2 max);
  void ToggleViewMode();

  // Aircraft synchronization
  void ResetMainState();
  void UpdateWorldOrigin(const sim::Aircraft &aircraft);
  AircraftSnapshot CaptureAircraft(const sim::Aircraft &aircraft) const;
  Vec3 ProjectWorldPosition(const sim::Aircraft &aircraft) const;
  void SyncFlightPath(const sim::Aircraft &source);
  void SyncGroundScroll(const sim::AircraftState &state);
  void UpdateSnapshotViewState();

  struct MotionState {
    Vec3 groundScroll{};
    double lastSampleTimeSec = 0.0;
    bool hasSample = false;
  };

  struct WorldOrigin {
    double latitudeRad = 0.0;
    double longitudeRad = 0.0;
    double radiusFt = 0.0;
    bool initialized = false;
  };

  // Scene state
  Scene scene_;
  CameraComponent *mainCamera_ = nullptr;
  FrameSnapshot snapshot_{};

  // Non-owning simulation sources
  const sim::Simulation *const mainSimulation_;
  const sim::Simulation *const shadowSimulation_;

  // View state
  ViewMode viewMode_ = ViewMode::Orbit;
  ViewOptions viewOptions_{};
  bool shadowEnabled_ = false;
  bool minimapMinimized_ = false;

  // Flight path
  FlightPathHistory flightPath_{};

  // Motion cache
  MotionState motion_{};

  // Fixed local-world projection
  WorldOrigin worldOrigin_{};
  std::optional<double> lastMainSimulationTimeSec_;
};
} // namespace viz
