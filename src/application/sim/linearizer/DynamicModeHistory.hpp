#pragma once

#include "application/sim/linearizer/DynamicModeAnalyzer.hpp"

#include <vector>

namespace gnc {
struct DynamicModeSnapshot {
  double simulationTimeSec = 0.0;
  DynamicModeAnalysis analysis;
};

class DynamicModeHistory {
public:
  // Snapshot lifecycle
  void Push(DynamicModeSnapshot snapshot);
  void Clear();

  // Time-based lookup
  const DynamicModeSnapshot *FindLatestAtOrBefore(double timeSec) const;
  const std::vector<DynamicModeSnapshot> &GetSnapshots() const;

private:
  std::vector<DynamicModeSnapshot> snapshots_;
};
} // namespace gnc
