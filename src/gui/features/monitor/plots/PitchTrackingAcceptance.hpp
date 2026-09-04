#pragma once

namespace gui {
struct PitchTrackingToleranceBand {
  double upperDeg = 0.0;
  double lowerDeg = 0.0;
};

constexpr PitchTrackingToleranceBand MakePitchTrackingToleranceBand(
    double commandedPitchDeg, double toleranceDeg) {
  return {
      .upperDeg = commandedPitchDeg + toleranceDeg,
      .lowerDeg = commandedPitchDeg - toleranceDeg,
  };
}
} // namespace gui
