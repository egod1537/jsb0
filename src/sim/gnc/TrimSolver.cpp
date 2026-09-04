#include "sim/gnc/TrimSolver.hpp"

#include "sim/Aircraft.hpp"
#include "sim/InitialCondition.hpp"
#include "sim/control/ControlInput.hpp"

#include <exception>
#include <iostream>

namespace {
using gnc::TrimMode;
using gnc::TrimRequest;
using gnc::TrimResult;

const char *TrimModeName(gnc::TrimMode mode) {
  switch (mode) {
  case gnc::TrimMode::Longitudinal:
    return "Longitudinal";
  case gnc::TrimMode::Full:
    return "Full";
  case gnc::TrimMode::Ground:
    return "Ground";
  }

  return "Unknown";
}

void PreparePropulsionForTrim(sim::Aircraft &aircraft, TrimMode mode) {
  if (mode == TrimMode::Ground) {
    return;
  }

  aircraft.GetEngines().StartAll();
}

TrimResult BuildTrimResult(const sim::Aircraft &aircraft) {
  const auto &properties = aircraft.GetProperties();
  const auto &controls = aircraft.GetControls();
  const control::ControlInput appliedInput = controls.GetAppliedInput();

  TrimResult result{};
  result.success = true;
  result.alphaRad = properties.Alpha().Rad();
  result.betaRad = properties.Beta().Rad();
  result.rollRad = properties.Roll().Rad();
  result.pitchRad = properties.Pitch().Rad();

  result.throttle = appliedInput.throttle;
  result.elevator = appliedInput.elevator;
  result.pitchTrim = controls.GetPitchTrim();
  result.aileron = appliedInput.aileron;
  result.rudder = appliedInput.rudder;

  result.uDotMps2 = properties.U().DotMps2();
  result.vDotMps2 = properties.V().DotMps2();
  result.wDotMps2 = properties.W().DotMps2();
  result.pDotRadPerSec2 = properties.P().DotRadPerSec2();
  result.qDotRadPerSec2 = properties.Q().DotRadPerSec2();
  result.rDotRadPerSec2 = properties.R().DotRadPerSec2();

  return result;
}

TrimResult ExecuteTrim(sim::Aircraft &aircraft, TrimMode mode,
    const TrimRequest *initialConditionRequest) {
  auto &properties = aircraft.GetProperties();

  std::cout << "[Trim] begin mode=" << TrimModeName(mode)
            << " simTime=" << properties.SimTime().Sec() << '\n';

  try {
    if (initialConditionRequest != nullptr) {
      if (!aircraft.InitializeForTrim(*initialConditionRequest)) {
        std::cout << "[Trim] RunIC failed simTime="
                  << properties.SimTime().Sec() << '\n';
        return {
            .success = false,
            .message = "Failed to apply initial conditions.",
        };
      }

      std::cout << "[Trim] RunIC simTime=" << properties.SimTime().Sec()
                << '\n';
    }

    PreparePropulsionForTrim(aircraft, mode);

    aircraft.RunTrim(mode);

    std::cout << "[Trim] DoTrim simTime=" << properties.SimTime().Sec() << '\n';

    TrimResult result = BuildTrimResult(aircraft);

    std::cout << "[Trim] end success=true simTime="
              << properties.SimTime().Sec() << '\n';

    return result;
  } catch (const std::exception &e) {
    std::cout << "[Trim] end success=false simTime="
              << properties.SimTime().Sec() << " message=" << e.what() << '\n';

    TrimResult result{};
    result.success = false;
    result.message = e.what();

    return result;
  }
}
} // namespace

namespace gnc::trim_solver {
TrimResult Solve(sim::Aircraft &aircraft, const TrimRequest &req) {
  return ExecuteTrim(aircraft, req.mode, &req);
}

TrimResult SolveCurrentState(sim::Aircraft &aircraft, TrimMode mode) {
  const sim::InitialCondition currentCondition = aircraft.GetCurrentCondition();
  if (!aircraft.ApplyInitialCondition(currentCondition)) {
    return {
        .success = false,
        .message = "Failed to apply current state as initial condition.",
    };
  }

  return ExecuteTrim(aircraft, mode, nullptr);
}
} // namespace gnc::trim_solver
