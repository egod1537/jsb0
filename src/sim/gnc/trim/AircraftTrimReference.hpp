#pragma once

namespace sim {
class Aircraft;
}

namespace gnc {
struct TrimResult;

struct AircraftTrimReference {
  double altitudeAglM = 0.0;
  double calibratedAirspeedMps = 0.0;
  double throttle = 0.0;
  double elevator = 0.0;
  double aileron = 0.0;
  double rudder = 0.0;
};

AircraftTrimReference MakeAircraftTrimReference(const sim::Aircraft &aircraft,
    const TrimResult &result);
} // namespace gnc
