#pragma once

namespace sim {
struct AircraftState {
  double simulationTimeSec = 0.0;

  double altitudeAglM = 0.0;
  double altitudeAslM = 0.0;
  double calibratedAirspeedMps = 0.0;
  double trueAirspeedMps = 0.0;

  double rollRad = 0.0;
  double pitchRad = 0.0;
  double headingRad = 0.0;
  double courseRad = 0.0;
  double alphaRad = 0.0;
  double betaRad = 0.0;

  double uMps = 0.0;
  double vMps = 0.0;
  double wMps = 0.0;

  double pRadPerSec = 0.0;
  double qRadPerSec = 0.0;
  double rRadPerSec = 0.0;
};

struct AircraftStateDerivative {
  double uDotMps2 = 0.0;
  double vDotMps2 = 0.0;
  double wDotMps2 = 0.0;

  double pDotRadPerSec2 = 0.0;
  double qDotRadPerSec2 = 0.0;
  double rDotRadPerSec2 = 0.0;
};
} // namespace sim
