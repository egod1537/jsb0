#pragma once

namespace gui {
struct CourseTrackingToleranceBand {
  double upperDeg = 0.0;
  double lowerDeg = 0.0;
};

constexpr CourseTrackingToleranceBand MakeCourseTrackingToleranceBand(
    double commandedCourseDeg, double toleranceDeg) {
  return {
      .upperDeg = commandedCourseDeg + toleranceDeg,
      .lowerDeg = commandedCourseDeg - toleranceDeg,
  };
}
} // namespace gui
