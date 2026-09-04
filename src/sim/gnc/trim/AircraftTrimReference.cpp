#include "sim/gnc/trim/AircraftTrimReference.hpp"

#include "sim/Aircraft.hpp"
#include "sim/gnc/TrimTypes.hpp"

#include <algorithm>

namespace gnc {
AircraftTrimReference MakeAircraftTrimReference(const sim::Aircraft &aircraft,
    const TrimResult &result) {
  return AircraftTrimReference{
      .altitudeAglM = aircraft.GetProperties().AltitudeAgl().M(),
      .calibratedAirspeedMps =
          std::max(aircraft.GetProperties().CalibratedAirspeed().Mps(), 0.1),
      .throttle = result.throttle,
      .elevator = result.elevator,
      .aileron = result.aileron,
      .rudder = result.rudder,
  };
}
} // namespace gnc
