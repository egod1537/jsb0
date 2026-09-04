#pragma once

namespace gui {
struct MonitorConfig {
  double rollTrackingToleranceDeg = 0.1;
  double pitchTrackingToleranceDeg = 0.1;
  double courseTrackingToleranceDeg = 1.0;
};
} // namespace gui
