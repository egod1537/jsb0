#include "sim/gnc/TrimWorkflow.hpp"

#include "sim/Aircraft.hpp"
#include "sim/InitialCondition.hpp"
#include "sim/control/FlightControlManager.hpp"
#include "sim/gnc/TrimService.hpp"
namespace gnc {
TrimRequest TrimWorkflow::MakeRequest(
    const sim::InitialCondition &initialCondition, TrimMode mode) {
  TrimRequest request{};
  request.mode = mode;
  request.calibratedAirspeedMps = initialCondition.calibratedAirspeedMps;
  request.altitudeAslM = initialCondition.altitudeAslM;
  request.flightPathAngleRad = 0.0;
  return request;
}

bool TrimWorkflow::Execute(TrimService &trimService, sim::Aircraft &aircraft,
    control::FlightControlManager &flightControls, const TrimRequest &request,
    const TrimWorkflowOptions &options) {
  const bool computed =
      options.fromCurrentState
          ? trimService.ComputeCurrentState(aircraft, request.mode)
          : trimService.Compute(aircraft, request);
  if (!computed || !trimService.ApplyStored(aircraft)) {
    return false;
  }

  if (const TrimResult *result = trimService.GetResult()) {
    flightControls.SynchronizeWithTrimResult(aircraft, *result);
  }
  if (options.resetSimulationTime) {
    aircraft.ResetSimulationTime();
  }
  return true;
}
} // namespace gnc
