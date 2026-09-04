#pragma once

#include "common/Options.hpp"
#include "sim/Aircraft.hpp"
#include "sim/ErrorTracker.hpp"
#include "sim/InitialCondition.hpp"
#include "sim/Tick.hpp"
#include "sim/control/FlightControlManager.hpp"
#include "sim/gnc/TrimService.hpp"
#include "sim/telemetry/TelemetryRegistry.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace gnc {
class IAutopilot;
} // namespace gnc

namespace sim {
struct SimResetOptions {
  bool runTrim = true;
  gnc::TrimMode trimMode = gnc::TrimMode::Full;
  std::optional<FDMEnvironmentState> environment;
};

class Simulation {
public:
  // Lifecycle and execution
  explicit Simulation(std::unique_ptr<gnc::IAutopilot> autopilot);
  ~Simulation();

  Simulation(const Simulation &other) = delete;
  Simulation &operator=(const Simulation &other) = delete;

  bool Initialize(
      std::string_view aircraftName = opts::simulation::AircraftName,
      double simulationHz = opts::simulation::Hz);
  bool Tick();
  bool Step(double dtSec);
  void Shutdown();
  bool IsInitialized() const { return initialized_; }

  // Configuration and state
  const std::string &GetAircraftName() const;
  double GetSimulationHz() const;
  double GetTickSizeSec() const;
  double GetTime() const;

  // Reset and initial condition
  bool Reset();
  bool Reset(const InitialCondition &initialCondition);
  bool Reset(const InitialCondition &initialCondition,
      const SimResetOptions &options);
  InitialCondition GetCurrentCondition() const;
  const InitialCondition &GetDefaultInitialCondition() const;

  // Domain services
  gnc::TrimService &GetTrimService();
  const gnc::TrimService &GetTrimService() const;

  Aircraft &GetAircraft();
  const Aircraft &GetAircraft() const;

  control::FlightControlManager &GetFlightControlManager();
  const control::FlightControlManager &GetFlightControlManager() const;

  telemetry::TelemetryRegistry &GetTelemetryRegistry();
  const telemetry::TelemetryRegistry &GetTelemetryRegistry() const;
  telemetry::TelemetryRegistry &GetTelemetry();
  const telemetry::TelemetryRegistry &GetTelemetry() const;

  // Error reporting
  ErrorTracker &GetErrorTracker();
  const ErrorTracker &GetErrorTracker() const;

private:
  // Step pipeline
  bool ProcessStep(double dtSec);
  sim::Tick MakeTick(double dtSec) const;

  // Reset preparation
  bool ApplyInitialTrim(const InitialCondition &initialCondition,
      gnc::TrimMode mode = gnc::TrimMode::Full);
  void ApplyEnvironment(const FDMEnvironmentState &environment);

  // Configuration
  std::string aircraftName_;
  double simulationHz_ = opts::simulation::Hz;

  // Runtime state
  bool initialized_ = false;
  InitialCondition defaultInitialCondition_;
  gnc::TrimService trimService_;
  std::uint64_t tickIndex_ = 0;

  // Owned simulation data
  Aircraft aircraft_;
  control::FlightControlManager flightControlManager_;
  telemetry::TelemetryRegistry telemetryRegistry_;

  // Diagnostics
  ErrorTracker errorTracker_;
};
} // namespace sim
