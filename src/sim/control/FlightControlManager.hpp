#pragma once

#include "sim/control/ControlInput.hpp"
#include "sim/control/FlightControlMode.hpp"
#include "sim/control/ManualFlightControlController.hpp"
#include "sim/gnc/autopilot/IAutopilot.hpp"
#include "sim/gnc/TrimTypes.hpp"

#include <memory>
#include <optional>

namespace sim {
class Aircraft;
struct Tick;
} // namespace sim

namespace control {
class FlightControlManager final {
public:
  explicit FlightControlManager(std::unique_ptr<gnc::IAutopilot> autopilot);

  // Simulation stepping
  void Tick(sim::Aircraft &aircraft, const sim::Tick &tick);

  // Active source
  FlightControlMode GetMode() const;
  void SetMode(FlightControlMode mode);

  // Owned controllers
  ManualFlightControlController &GetManualController();
  const ManualFlightControlController &GetManualController() const;
  gnc::IAutopilot &GetAutopilot();
  const gnc::IAutopilot &GetAutopilot() const;

  // Controller state
  void ResetControllers();
  void SynchronizeWithTrimResult(sim::Aircraft &aircraft,
      const gnc::TrimResult &trimResult);

private:
  // Control routing
  std::optional<ControlInput> ProduceControlInput(sim::Aircraft &aircraft,
      const sim::Tick &tick);

  // Control sources
  ManualFlightControlController manualController_;
  std::unique_ptr<gnc::IAutopilot> autopilot_;

  // Routing state
  FlightControlMode mode_ = FlightControlMode::Manual;
};
} // namespace control
