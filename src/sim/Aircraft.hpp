#pragma once

#include "common/Options.hpp"
#include "sim/jsbsim/ControlSystem.hpp"
#include "sim/jsbsim/EngineSystem.hpp"
#include "sim/jsbsim/FDMStateAccess.hpp"
#include "sim/jsbsim/Properties.hpp"
#include "sim/AircraftState.hpp"
#include "sim/FDMState.hpp"
#include "sim/InitialCondition.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>

namespace JSBSim {
class FGFDMExec;
} // namespace JSBSim

namespace gnc {
enum class TrimMode;
struct TrimRequest;
struct LinearizationResult;
} // namespace gnc

namespace sim {
class Aircraft {
public:
  // Lifetime and stepping
  Aircraft();
  ~Aircraft();
  Aircraft(const Aircraft &other) = delete;
  Aircraft &operator=(const Aircraft &other) = delete;
  bool Initialize(const InitialCondition &initialCondition);
  bool Initialize(std::string_view aircraftName, double simulationHz,
      const InitialCondition &initialCondition);
  bool Tick();
  bool Step(double dtSec);

  // Configuration
  const std::string &GetAircraftName() const;
  double GetSimulationHz() const;

  // Initial condition and reset
  bool ApplyInitialCondition(const InitialCondition &initialCondition);
  void SetInitialConditionInputs(const InitialCondition &initialCondition);
  InitialCondition GetCurrentCondition() const;
  bool Reset(const InitialCondition &initialCondition);
  void ResetSimulationTime();

  // Trim operations
  bool InitializeForTrim(const gnc::TrimRequest &request);
  void RunTrim(gnc::TrimMode mode);

  // Aircraft state
  AircraftState GetAircraftState() const;
  AircraftStateDerivative GetAircraftStateDerivative() const;

  // Flight-dynamics state synchronization
  FDMState ExtractFDMState(FDMStateFlags flags) const;
  void ApplyFDMState(const FDMState &state);
  void SetIntegrationSuspended(bool suspended);
  bool IsIntegrationSuspended() const;

  // Linearization
  gnc::LinearizationResult ComputeLinearization();

  // Flight model interfaces
  jsbsim::ControlSystem &GetControls();
  const jsbsim::ControlSystem &GetControls() const;
  jsbsim::EngineSystem &GetEngines();
  const jsbsim::EngineSystem &GetEngines() const;
  jsbsim::Properties &GetProperties();
  const jsbsim::Properties &GetProperties() const;

private:
  // JSBSim setup
  void ConfigurePaths();
  void ConfigureOutputPath();
  void RemoveOutputDirectory();
  bool LoadAircraft(std::string_view aircraftName);
  void ConfigureSimulation(double simulationHz);
  void DisableExternalOutput();
  void PrepareExternalOutputForReset();
  bool InitializeState();

  // Configuration
  std::string aircraftName_;
  double simulationHz_ = opts::simulation::Hz;
  std::filesystem::path outputDirectory_;

  // JSBSim dependencies
  std::unique_ptr<JSBSim::FGFDMExec> fdm_;
  jsbsim::FDMStateAccess fdmStateAccess_;
  jsbsim::ControlSystem controls_;
  jsbsim::EngineSystem engines_;
  jsbsim::Properties properties_;
};
} // namespace sim
