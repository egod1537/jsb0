#pragma once

#include "sim/FDMState.hpp"
#include "sim/InitialCondition.hpp"
#include "sim/linearization/LinearizationResult.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace sim {
class AsyncAircraftLinearizer {
public:
  struct Completion {
    std::uint64_t generation{};
    std::optional<gnc::LinearizationResult> linearization;
    std::string errorMessage;
  };

  AsyncAircraftLinearizer();
  ~AsyncAircraftLinearizer();

  AsyncAircraftLinearizer(const AsyncAircraftLinearizer &other) = delete;
  AsyncAircraftLinearizer &operator=(
      const AsyncAircraftLinearizer &other) = delete;

  bool Submit(std::uint64_t generation, std::string_view aircraftName,
      double simulationHz, const InitialCondition &initialCondition,
      FDMState sourceState);
  bool IsBusy() const;
  std::optional<Completion> TakeCompletion();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace sim
