#include "flightui/visualization/components/FlightCameraController.hpp"
#include "flightui/visualization/components/TelemetryOverlay.hpp"
#include "gui/features/flightviz/FlightVisualizer.hpp"

#include "common/math/Math.hpp"
#include "flightui/visualization/render/CameraComponent.hpp"

#include <cassert>

namespace {
constexpr viz::Vec3 AircraftPosition{0.0F, 0.0F, 0.35F};

void RequireAircraftTracked(float visualAltitude) {
  viz::CameraComponent camera;
  viz::FlightCameraController controller;
  controller.SetCamera(&camera);

  viz::FrameSnapshot snapshot{};
  snapshot.viewMode = viz::ViewMode::ThirdPerson;
  snapshot.aircraft.available = true;
  snapshot.aircraft.position = AircraftPosition;
  snapshot.aircraft.visualAltitude = visualAltitude;
  snapshot.aircraft.state.headingRad = math::DegToRad(37.0);
  snapshot.aircraft.state.pitchRad = math::DegToRad(5.0);
  snapshot.shadowEnabled = true;
  snapshot.shadowAircraft.available = true;
  snapshot.shadowAircraft.position = {1000.0F, 1000.0F, 1000.0F};
  controller.OnTick(viz::TickContext{snapshot});

  const viz::CameraView view = camera.BuildView();
  const viz::Vec3 directionToAircraft =
      viz::Normalize(AircraftPosition - view.eye);

  assert(viz::Dot(view.forward, directionToAircraft) > 0.98F);
}

void RequireVisualizerViewModesAreIndependent() {
  viz::FlightVisualizer primary;
  viz::FlightVisualizer baseline;
  assert(primary.GetViewMode() == viz::ViewMode::Orbit);
  assert(baseline.GetViewMode() == viz::ViewMode::Orbit);

  primary.SetViewMode(viz::ViewMode::ThirdPerson);
  assert(primary.GetViewMode() == viz::ViewMode::ThirdPerson);
  assert(baseline.GetViewMode() == viz::ViewMode::Orbit);
}

void RequireHudUsesSiFlightStateAndDegreeAngles() {
  sim::AircraftState state;
  state.altitudeAglM = 304.8;
  state.calibratedAirspeedMps = 41.1556;
  state.trueAirspeedMps = 41.8;
  state.rollRad = math::DegToRad(-0.2);
  state.pitchRad = math::DegToRad(2.2);
  state.headingRad = math::DegToRad(360.0);
  state.courseRad = 0.0;

  assert(viz::FormatTelemetryFlightState(state)
         == "Alt AGL 304.8 m  Course 0.0 deg  CAS 41.2 m/s  TAS 41.8 m/s");
  assert(viz::FormatTelemetryAttitude(state)
         == "Roll -0.2 deg  Pitch 2.2 deg  Heading 360.0 deg");
}
} // namespace

int main() {
  RequireAircraftTracked(0.35F);
  RequireAircraftTracked(52.0F);
  RequireVisualizerViewModesAreIndependent();
  RequireHudUsesSiFlightStateAndDegreeAngles();
  return 0;
}
