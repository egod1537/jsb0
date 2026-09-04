#include "sim/Simulation.hpp"

#include "sim/gnc/TrimWorkflow.hpp"
#include "sim/gnc/autopilot/IAutopilot.hpp"
#include "sim/telemetry/SimTelemetryPublisher.hpp"
#include "common/math/Math.hpp"

#include <cmath>
#include <iostream>
#include <numbers>
#include <utility>

namespace sim {
Simulation::Simulation(std::unique_ptr<gnc::IAutopilot> autopilot)
    : flightControlManager_(std::move(autopilot)) {}

Simulation::~Simulation() = default;

bool Simulation::Initialize(std::string_view aircraftName,
    double simulationHz) {
  if (initialized_) {
    return true;
  }

  aircraftName_ = aircraftName;
  simulationHz_ = simulationHz;
  defaultInitialCondition_ = InitialCondition{};
  tickIndex_ = 0;
  telemetryRegistry_.Clear();
  errorTracker_.ClearError();

  flightControlManager_.ResetControllers();

  if (!aircraft_.Initialize(aircraftName_,
          simulationHz_,
          defaultInitialCondition_)) {
    errorTracker_.SetError("Failed to initialize aircraft.");
    return false;
  }
  if (!ApplyInitialTrim(defaultInitialCondition_)) {
    return false;
  }
  initialized_ = true;
  return true;
}

bool Simulation::Tick() { return Step(GetTickSizeSec()); }

bool Simulation::Step(double dtSec) {
  if (!initialized_) {
    errorTracker_.SetError("Simulation has not been initialized.");
    return false;
  }
  if (!std::isfinite(dtSec) || dtSec <= 0.0) {
    errorTracker_.SetError("Simulation step size must be finite and positive.");
    return false;
  }
  return ProcessStep(dtSec);
}

void Simulation::Shutdown() { initialized_ = false; }

const std::string &Simulation::GetAircraftName() const { return aircraftName_; }

double Simulation::GetSimulationHz() const { return simulationHz_; }

double Simulation::GetTickSizeSec() const { return 1.0 / simulationHz_; }

double Simulation::GetTime() const {
  return aircraft_.GetAircraftState().simulationTimeSec;
}

bool Simulation::Reset() { return Reset(defaultInitialCondition_); }

bool Simulation::Reset(const InitialCondition &initialCondition) {
  return Reset(initialCondition, SimResetOptions{});
}

bool Simulation::Reset(const InitialCondition &initialCondition,
    const SimResetOptions &options) {
  if (!initialized_) {
    errorTracker_.SetError("Simulation has not been initialized.");
    return false;
  }

  InitialCondition normalized = initialCondition;
  normalized.headingRad =
      math::Wrap(normalized.headingRad, 0.0, 2.0 * std::numbers::pi_v<double>);

  std::string validationError;
  if (!ValidateInitialCondition(normalized, &validationError)) {
    errorTracker_.SetError(validationError);
    return false;
  }

  if (!aircraft_.Reset(normalized)) {
    errorTracker_.SetError(
        "Failed to reset aircraft with the requested initial condition.");
    return false;
  }
  if (options.environment.has_value()) {
    ApplyEnvironment(*options.environment);
  }

  flightControlManager_.ResetControllers();
  if (options.runTrim) {
    if (!ApplyInitialTrim(normalized, options.trimMode)) {
      return false;
    }
  } else {
    trimService_.Clear();
    aircraft_.ResetSimulationTime();
  }
  if (options.environment.has_value()) {
    ApplyEnvironment(*options.environment);
  }

  tickIndex_ = 0;
  telemetryRegistry_.Clear();
  errorTracker_.ClearError();
  return true;
}

InitialCondition Simulation::GetCurrentCondition() const {
  return aircraft_.GetCurrentCondition();
}

const InitialCondition &Simulation::GetDefaultInitialCondition() const {
  return defaultInitialCondition_;
}

void Simulation::ApplyEnvironment(const FDMEnvironmentState &environment) {
  FDMState state{
      .flags = FDMStateFlags::Environment,
      .environment = environment,
  };
  aircraft_.ApplyFDMState(state);
}

gnc::TrimService &Simulation::GetTrimService() { return trimService_; }

const gnc::TrimService &Simulation::GetTrimService() const {
  return trimService_;
}

Aircraft &Simulation::GetAircraft() { return aircraft_; }

const Aircraft &Simulation::GetAircraft() const { return aircraft_; }

control::FlightControlManager &Simulation::GetFlightControlManager() {
  return flightControlManager_;
}

const control::FlightControlManager &
Simulation::GetFlightControlManager() const {
  return flightControlManager_;
}

telemetry::TelemetryRegistry &Simulation::GetTelemetryRegistry() {
  return telemetryRegistry_;
}

const telemetry::TelemetryRegistry &Simulation::GetTelemetryRegistry() const {
  return telemetryRegistry_;
}

telemetry::TelemetryRegistry &Simulation::GetTelemetry() {
  return telemetryRegistry_;
}

const telemetry::TelemetryRegistry &Simulation::GetTelemetry() const {
  return telemetryRegistry_;
}

ErrorTracker &Simulation::GetErrorTracker() { return errorTracker_; }

const ErrorTracker &Simulation::GetErrorTracker() const {
  return errorTracker_;
}

bool Simulation::ProcessStep(double dtSec) {
  const sim::Tick tick = MakeTick(dtSec);
  flightControlManager_.Tick(aircraft_, tick);
  if (!aircraft_.Step(dtSec)) {
    errorTracker_.SetError("JSBSim simulation stopped.");
    std::cerr << errorTracker_.GetLastError().value() << '\n';
    return false;
  }

  const sim::Tick postTick = MakeTick(dtSec);
  telemetry::SimTelemetryPublisher::Publish(aircraft_,
      flightControlManager_,
      postTick,
      telemetryRegistry_);
  ++tickIndex_;
  return true;
}

sim::Tick Simulation::MakeTick(double dtSec) const {
  return sim::Tick{tickIndex_,
      dtSec,
      aircraft_.GetAircraftState().simulationTimeSec};
}

bool Simulation::ApplyInitialTrim(const InitialCondition &initialCondition,
    gnc::TrimMode mode) {
  const bool trimmed = gnc::TrimWorkflow::Execute(trimService_,
      aircraft_,
      flightControlManager_,
      gnc::TrimWorkflow::MakeRequest(initialCondition, mode),
      {.resetSimulationTime = true});
  if (!trimmed) {
    errorTracker_.SetError("Initial trim failed.");
    std::cerr << "Initial trim failed: " << errorTracker_.GetLastError().value()
              << '\n';
    return false;
  }
  return true;
}
} // namespace sim
