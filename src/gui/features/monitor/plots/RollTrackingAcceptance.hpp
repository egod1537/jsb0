#pragma once

namespace gui {
struct RollTrackingAcceptance {
  double settlingUpperDeg = 0.0;
  double settlingLowerDeg = 0.0;
  double overshootLimitDeg = 0.0;
  double undershootLimitDeg = 0.0;
};

constexpr RollTrackingAcceptance MakeRollTrackingAcceptance(
    double commandedRollDeg, double settlingToleranceDeg,
    double trackingToleranceDeg) {
  return {
      .settlingUpperDeg = commandedRollDeg + settlingToleranceDeg,
      .settlingLowerDeg = commandedRollDeg - settlingToleranceDeg,
      .overshootLimitDeg = commandedRollDeg + trackingToleranceDeg,
      .undershootLimitDeg = commandedRollDeg - trackingToleranceDeg,
  };
}
} // namespace gui
