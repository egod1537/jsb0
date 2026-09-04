#include "sim/Simulation.hpp"

#include "sim/StateLogger.hpp"
#include "sim/gnc/TrimWorkflow.hpp"
#include "sim/gnc/autopilot/IAutopilot.hpp"
#include "sim/telemetry/SimulationTelemetryPublisher.hpp"
#include "common/math/Math.hpp"

#include <cmath>
#include <iostream>
#include <numbers>
#include <utility>

namespace sim {
Simulation::Simulation(std::unique_ptr<gnc::IAutopilot> autopilot) {
  AddComponent<control::FlightControlManager>(std::move(autopilot));
  AddComponent<StateLogger>();
}

Simulation::~Simulation() = default;

bool Simulation::Initialize(const SimulationConfig &config) {
  if (initialized_) {
    return true;
  }

  config_ = config;
  defaultInitialCondition_ = InitialCondition{};
  tickIndex_ = 0;
  telemetryRegistry_.Clear();
  errorTracker_.ClearError();

  auto *flightControlManager = GetComponent<control::FlightControlManager>();
  if (flightControlManager == nullptr) {
    errorTracker_.SetError("Flight control component is missing.");
    return false;
  }
  flightControlManager->ResetControllers();

  if (!aircraft_.Initialize(config_, defaultInitialCondition_)) {
    errorTracker_.SetError("Failed to initialize aircraft.");
    return false;
  }
  if (!ApplyInitialTrim(defaultInitialCondition_)) {
    return false;
  }
  if (!InitializeComponents()) {
    errorTracker_.SetErrorIfEmpty("Failed to initialize components.");
    ShutdownComponents();
    return false;
  }

  initialized_ = true;
  return true;
}

bool Simulation::Tick() { return Step(config_.GetDT()); }

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

void Simulation::Shutdown() {
  if (initialized_) {
    ShutdownComponents();
  }
  initialized_ = false;
}

const SimulationConfig &Simulation::GetConfig() const { return config_; }

double Simulation::GetTickSizeSec() const { return config_.GetDT(); }

double Simulation::GetTime() const {
  return aircraft_.GetAircraftState().simulationTimeSec;
}

bool Simulation::Reset() { return Reset(defaultInitialCondition_); }

bool Simulation::Reset(const InitialCondition &initialCondition) {
  return Reset(initialCondition, SimulationResetOptions{});
}

bool Simulation::Reset(const InitialCondition &initialCondition,
    const SimulationResetOptions &options) {
  if (!initialized_) {
    errorTracker_.SetError("Simulation has not been initialized.");
    return false;
  }

  InitialCondition normalized = initialCondition;
  normalized.headingRad = math::Wrap(normalized.headingRad,
      0.0,
      2.0 * std::numbers::pi_v<double>);

  std::string validationError;
  if (!ValidateInitialCondition(normalized, &validationError)) {
    errorTracker_.SetError(validationError);
    return false;
  }

  auto *flightControlManager = GetComponent<control::FlightControlManager>();
  if (flightControlManager == nullptr) {
    errorTracker_.SetError("Flight control component is missing.");
    return false;
  }
  if (!aircraft_.Reset(config_, normalized)) {
    errorTracker_.SetError(
        "Failed to reset aircraft with the requested initial condition.");
    return false;
  }
  if (options.environment.has_value()) {
    ApplyEnvironment(*options.environment);
  }

  flightControlManager->ResetControllers();
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
  if (!ResetComponents()) {
    errorTracker_.SetErrorIfEmpty("Failed to reset components.");
    return false;
  }

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
  if (!RunPreTickComponents(tick)) {
    errorTracker_.SetErrorIfEmpty("Simulation pre-tick component failed.");
    return false;
  }

  auto *flightControlManager = GetComponent<control::FlightControlManager>();
  if (flightControlManager == nullptr) {
    errorTracker_.SetError("Flight control component is missing.");
    return false;
  }
  if (!TickComponents(tick)) {
    errorTracker_.SetErrorIfEmpty("Simulation tick component failed.");
    return false;
  }
  if (!aircraft_.Step(dtSec)) {
    errorTracker_.SetError("JSBSim simulation stopped.");
    std::cerr << errorTracker_.GetLastError().value() << '\n';
    return false;
  }

  const sim::Tick postTick = MakeTick(dtSec);
  if (!RunPostTickComponents(postTick)) {
    errorTracker_.SetErrorIfEmpty("Simulation post-tick component failed.");
    return false;
  }
  telemetry::SimulationTelemetryPublisher::Publish(aircraft_,
      *flightControlManager,
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
  auto *flightControlManager = GetComponent<control::FlightControlManager>();
  if (flightControlManager == nullptr) {
    errorTracker_.SetError("Flight control component is missing.");
    return false;
  }
  const bool trimmed = gnc::TrimWorkflow::Execute(trimService_,
      aircraft_,
      *flightControlManager,
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

bool Simulation::InitializeComponent(Component &component) {
  if (component.initialized_) {
    return true;
  }
  if (!component.OnInitialize()) {
    component.OnShutdown();
    return false;
  }
  component.initialized_ = true;
  return true;
}

bool Simulation::InitializeComponents() {
  for (std::size_t index = 0; index < components_.size(); ++index) {
    if (!InitializeComponent(*components_[index])) {
      return false;
    }
  }
  return true;
}

bool Simulation::ResetComponents() {
  for (const auto &component : components_) {
    if (component->initialized_ && !component->OnReset()) {
      return false;
    }
  }
  return true;
}

bool Simulation::RunPreTickComponents(const sim::Tick &tick) {
  for (const auto &component : components_) {
    if (component->initialized_ && !component->OnPreTick(tick)) {
      return false;
    }
  }
  return true;
}

bool Simulation::TickComponents(const sim::Tick &tick) {
  for (const auto &component : components_) {
    if (component->initialized_ && !component->OnTick(tick)) {
      return false;
    }
  }
  return true;
}

bool Simulation::RunPostTickComponents(const sim::Tick &tick) {
  for (const auto &component : components_) {
    if (component->initialized_ && !component->OnPostTick(tick)) {
      return false;
    }
  }
  return true;
}

void Simulation::ShutdownComponents() {
  for (auto iterator = components_.rbegin(); iterator != components_.rend();
      ++iterator) {
    if ((*iterator)->initialized_) {
      (*iterator)->OnShutdown();
      (*iterator)->initialized_ = false;
    }
  }
}

Component *Simulation::FindComponent(const std::type_info &type) {
  for (const auto &component : components_) {
    if (typeid(*component) == type) {
      return component.get();
    }
  }
  return nullptr;
}

const Component *Simulation::FindComponent(const std::type_info &type) const {
  for (const auto &component : components_) {
    if (typeid(*component) == type) {
      return component.get();
    }
  }
  return nullptr;
}
} // namespace sim
