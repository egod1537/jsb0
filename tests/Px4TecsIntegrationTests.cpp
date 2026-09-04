#include "common/math/Math.hpp"
#include "sim/InitialCondition.hpp"
#include "sim/Simulation.hpp"
#include "sim/control/FlightControlManager.hpp"
#include "sim/control/FlightControlMode.hpp"
#include "sim/gnc/TrimTypes.hpp"
#include "sim/gnc/autopilot/PX4Autopilot.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
constexpr double SimHz = 120.0;
constexpr double CommandTimeSec = 5.0;
constexpr double RunDurationSec = 70.0;
constexpr double TailWindowSec = 10.0;
constexpr double MaximumTailPitchRangeDeg = 1.0;
constexpr double MaximumTailThrottleRange = 0.05;
constexpr double MaximumTailSaturationDurationSec = 0.1;
constexpr double MaximumPitchTrackingErrorDeg = 2.0;

enum class ScenarioKind {
  Level,
  Climb,
  Descent,
  AirspeedIncrease,
  AirspeedDecrease,
  Combined,
  Underspeed,
};

struct ScenarioDefinition {
  const char *name;
  ScenarioKind kind;
  double altitudeDeltaM;
  double airspeedDeltaMps;
};

struct Sample {
  double timeSec;
  double altitudeM;
  double airspeedMps;
  double pitchDeg;
  double targetPitchDeg;
  double throttle;
  double elevator;
  bool underspeed;
  bool pitchLimited;
  bool throttleSaturated;
};

struct Metrics {
  double tailAltitudeMeanErrorM = 0.0;
  double tailAltitudeMaximumErrorM = 0.0;
  double tailAltitudeRmsErrorM = 0.0;
  double tailAirspeedMeanErrorMps = 0.0;
  double tailAirspeedMaximumErrorMps = 0.0;
  double tailAirspeedRmsErrorMps = 0.0;
  double altitudeOvershootM = 0.0;
  double settlingTimeSec = std::numeric_limits<double>::infinity();
  double minimumAirspeedMps = std::numeric_limits<double>::infinity();
  double maximumAirspeedMps = -std::numeric_limits<double>::infinity();
  double minimumPitchDeg = std::numeric_limits<double>::infinity();
  double maximumPitchDeg = -std::numeric_limits<double>::infinity();
  double minimumTargetPitchDeg = std::numeric_limits<double>::infinity();
  double minimumThrottle = std::numeric_limits<double>::infinity();
  double maximumThrottle = -std::numeric_limits<double>::infinity();
  double tailAltitudeRangeM = 0.0;
  double tailAirspeedRangeMps = 0.0;
  double tailPitchRangeDeg = 0.0;
  double tailThrottleRange = 0.0;
  double tailPitchTrackingErrorDeg = 0.0;
  double elevatorSaturationDurationSec = 0.0;
  double throttleSaturationDurationSec = 0.0;
  double tailElevatorSaturationDurationSec = 0.0;
  double tailThrottleSaturationDurationSec = 0.0;
  double tailPitchLimitDurationSec = 0.0;
  double preCommandTargetPitchDeg = 0.0;
  double preCommandThrottle = 0.0;
  double minimumProtectedTargetPitchDeg =
      std::numeric_limits<double>::infinity();
  double maximumProtectedThrottle = 0.0;
  double underspeedActivationTimeSec = std::numeric_limits<double>::infinity();
  bool underspeedObserved = false;
};

struct TuningOverrides {
  std::optional<double> altitudeErrorGain;
  std::optional<double> airspeedErrorGain;
  std::optional<double> throttleDampingGain;
  std::optional<double> throttleIntegralGain;
  std::optional<double> pitchDampingGain;
  std::optional<double> pitchIntegralGain;
  std::optional<double> energyBalanceFeedForwardGain;
  std::optional<double> totalEnergyRateFilterTimeConstantSec;
  std::optional<double> pitchSlewRateDegPerSec;
  std::optional<double> throttleSlewRatePerSec;
  std::optional<double> maximumClimbRateMps;
  std::optional<double> maximumSinkRateMps;
};

struct Options {
  std::optional<ScenarioKind> scenario;
  TuningOverrides tuning;
};

control::FlightControlManager &Manager(sim::Simulation &simulation) {
  return simulation.GetFlightControlManager();
}

double Mean(const std::vector<double> &values) {
  double total = 0.0;
  for (double value : values) {
    total += value;
  }
  return values.empty() ? 0.0 : total / static_cast<double>(values.size());
}

double MaximumAbsolute(const std::vector<double> &values) {
  double maximum = 0.0;
  for (const double value : values) {
    maximum = std::max(maximum, std::abs(value));
  }
  return maximum;
}

double RootMeanSquare(const std::vector<double> &values) {
  double sumSquares = 0.0;
  for (const double value : values) {
    sumSquares += value * value;
  }
  return values.empty()
             ? 0.0
             : std::sqrt(sumSquares / static_cast<double>(values.size()));
}

double PeakToPeak(const std::vector<double> &values) {
  if (values.empty()) {
    return 0.0;
  }
  const auto [minimum, maximum] =
      std::minmax_element(values.begin(), values.end());
  return *maximum - *minimum;
}

std::pair<double, double> SettlingTolerances(
    const ScenarioDefinition &scenario) {
  switch (scenario.kind) {
  case ScenarioKind::Level:
    return {1.0, 0.3};
  case ScenarioKind::Climb:
  case ScenarioKind::Descent:
    return {2.0, 0.5};
  case ScenarioKind::AirspeedIncrease:
  case ScenarioKind::AirspeedDecrease:
  case ScenarioKind::Combined:
  case ScenarioKind::Underspeed:
    return {2.0, 0.3};
  }
  return {2.0, 0.5};
}

double CalculateSettlingTime(const std::vector<Sample> &samples,
    double targetAltitudeM, double targetAirspeedMps,
    const ScenarioDefinition &scenario) {
  const auto [altitudeToleranceM, airspeedToleranceMps] =
      SettlingTolerances(scenario);
  double lastViolationTimeSec = CommandTimeSec;
  bool violationObserved = false;
  for (const Sample &sample : samples) {
    if (sample.timeSec < CommandTimeSec) {
      continue;
    }
    const bool outsideBand =
        std::abs(targetAltitudeM - sample.altitudeM) > altitudeToleranceM
        || std::abs(targetAirspeedMps - sample.airspeedMps)
               > airspeedToleranceMps;
    if (outsideBand) {
      lastViolationTimeSec = sample.timeSec;
      violationObserved = true;
    }
  }
  if (!violationObserved) {
    return 0.0;
  }
  if (lastViolationTimeSec >= RunDurationSec - 1.0 / SimHz) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0,
      lastViolationTimeSec - CommandTimeSec + 1.0 / SimHz);
}

void ApplyTuningOverrides(gnc::Px4TecsSettings &settings,
    const TuningOverrides &overrides) {
  const auto assign = [](double &target, const std::optional<double> &value) {
    if (value) {
      target = *value;
    }
  };
  assign(settings.altitudeErrorGain, overrides.altitudeErrorGain);
  assign(settings.airspeedErrorGain, overrides.airspeedErrorGain);
  assign(settings.throttleDampingGain, overrides.throttleDampingGain);
  assign(settings.throttleIntegralGain, overrides.throttleIntegralGain);
  assign(settings.pitchDampingGain, overrides.pitchDampingGain);
  assign(settings.pitchIntegralGain, overrides.pitchIntegralGain);
  assign(settings.energyBalanceFeedForwardGain,
      overrides.energyBalanceFeedForwardGain);
  assign(settings.totalEnergyRateFilterTimeConstantSec,
      overrides.totalEnergyRateFilterTimeConstantSec);
  if (overrides.pitchSlewRateDegPerSec) {
    settings.pitchSlewRateRadPerSec =
        math::DegToRad(*overrides.pitchSlewRateDegPerSec);
  }
  assign(settings.throttleSlewRatePerSec, overrides.throttleSlewRatePerSec);
  assign(settings.maximumClimbRateMps, overrides.maximumClimbRateMps);
  assign(settings.maximumSinkRateMps, overrides.maximumSinkRateMps);
}

Metrics Evaluate(const std::vector<Sample> &samples, double targetAltitudeM,
    double targetAirspeedMps, double initialAltitudeM,
    const ScenarioDefinition &scenario) {
  Metrics metrics;
  std::vector<double> tailAltitudeError;
  std::vector<double> tailAirspeedError;
  std::vector<double> tailAltitude;
  std::vector<double> tailAirspeed;
  std::vector<double> tailPitch;
  std::vector<double> tailThrottle;
  std::vector<double> tailTrackingError;
  std::size_t saturatedElevatorSamples = 0;
  std::size_t saturatedThrottleSamples = 0;
  std::size_t tailSaturatedElevatorSamples = 0;
  std::size_t tailSaturatedThrottleSamples = 0;
  std::size_t tailPitchLimitedSamples = 0;

  for (const Sample &sample : samples) {
    if (sample.timeSec < CommandTimeSec) {
      metrics.preCommandTargetPitchDeg = sample.targetPitchDeg;
      metrics.preCommandThrottle = sample.throttle;
    }
    if (sample.timeSec < CommandTimeSec) {
      continue;
    }
    metrics.minimumAirspeedMps =
        std::min(metrics.minimumAirspeedMps, sample.airspeedMps);
    metrics.maximumAirspeedMps =
        std::max(metrics.maximumAirspeedMps, sample.airspeedMps);
    metrics.minimumPitchDeg =
        std::min(metrics.minimumPitchDeg, sample.pitchDeg);
    metrics.maximumPitchDeg =
        std::max(metrics.maximumPitchDeg, sample.pitchDeg);
    metrics.minimumTargetPitchDeg =
        std::min(metrics.minimumTargetPitchDeg, sample.targetPitchDeg);
    metrics.minimumThrottle =
        std::min(metrics.minimumThrottle, sample.throttle);
    metrics.maximumThrottle =
        std::max(metrics.maximumThrottle, sample.throttle);
    saturatedElevatorSamples += std::abs(sample.elevator) >= 0.999 ? 1u : 0u;
    saturatedThrottleSamples += sample.throttleSaturated ? 1u : 0u;
    metrics.underspeedObserved =
        metrics.underspeedObserved || sample.underspeed;
    if (sample.underspeed) {
      metrics.underspeedActivationTimeSec =
          std::min(metrics.underspeedActivationTimeSec, sample.timeSec);
      metrics.minimumProtectedTargetPitchDeg =
          std::min(metrics.minimumProtectedTargetPitchDeg,
              sample.targetPitchDeg);
      metrics.maximumProtectedThrottle =
          std::max(metrics.maximumProtectedThrottle, sample.throttle);
    }

    if (sample.timeSec >= RunDurationSec - TailWindowSec) {
      tailAltitude.push_back(sample.altitudeM);
      tailAirspeed.push_back(sample.airspeedMps);
      tailAltitudeError.push_back(targetAltitudeM - sample.altitudeM);
      tailAirspeedError.push_back(targetAirspeedMps - sample.airspeedMps);
      tailPitch.push_back(sample.pitchDeg);
      tailThrottle.push_back(sample.throttle);
      tailTrackingError.push_back(sample.targetPitchDeg - sample.pitchDeg);
      tailSaturatedElevatorSamples +=
          std::abs(sample.elevator) >= 0.999 ? 1u : 0u;
      tailSaturatedThrottleSamples += sample.throttleSaturated ? 1u : 0u;
      tailPitchLimitedSamples += sample.pitchLimited ? 1u : 0u;
    }
  }

  metrics.tailAltitudeMeanErrorM = Mean(tailAltitudeError);
  metrics.tailAltitudeMaximumErrorM = MaximumAbsolute(tailAltitudeError);
  metrics.tailAltitudeRmsErrorM = RootMeanSquare(tailAltitudeError);
  metrics.tailAirspeedMeanErrorMps = Mean(tailAirspeedError);
  metrics.tailAirspeedMaximumErrorMps = MaximumAbsolute(tailAirspeedError);
  metrics.tailAirspeedRmsErrorMps = RootMeanSquare(tailAirspeedError);
  metrics.settlingTimeSec = CalculateSettlingTime(samples,
      targetAltitudeM,
      targetAirspeedMps,
      scenario);
  if (scenario.altitudeDeltaM > 0.0) {
    double maximumAltitude = initialAltitudeM;
    for (const Sample &sample : samples) {
      maximumAltitude = std::max(maximumAltitude, sample.altitudeM);
    }
    metrics.altitudeOvershootM =
        std::max(0.0, maximumAltitude - targetAltitudeM);
  } else if (scenario.altitudeDeltaM < 0.0) {
    double minimumAltitude = initialAltitudeM;
    for (const Sample &sample : samples) {
      minimumAltitude = std::min(minimumAltitude, sample.altitudeM);
    }
    metrics.altitudeOvershootM =
        std::max(0.0, targetAltitudeM - minimumAltitude);
  }
  metrics.tailAltitudeRangeM = PeakToPeak(tailAltitude);
  metrics.tailAirspeedRangeMps = PeakToPeak(tailAirspeed);
  metrics.tailPitchRangeDeg = PeakToPeak(tailPitch);
  metrics.tailThrottleRange = PeakToPeak(tailThrottle);
  for (double error : tailTrackingError) {
    metrics.tailPitchTrackingErrorDeg =
        std::max(metrics.tailPitchTrackingErrorDeg, std::abs(error));
  }
  metrics.elevatorSaturationDurationSec =
      static_cast<double>(saturatedElevatorSamples) / SimHz;
  metrics.throttleSaturationDurationSec =
      static_cast<double>(saturatedThrottleSamples) / SimHz;
  metrics.tailElevatorSaturationDurationSec =
      static_cast<double>(tailSaturatedElevatorSamples) / SimHz;
  metrics.tailThrottleSaturationDurationSec =
      static_cast<double>(tailSaturatedThrottleSamples) / SimHz;
  metrics.tailPitchLimitDurationSec =
      static_cast<double>(tailPitchLimitedSamples) / SimHz;
  return metrics;
}

Metrics Execute(const ScenarioDefinition &scenario,
    const TuningOverrides &overrides) {
  sim::Simulation simulation(std::make_unique<gnc::PX4Autopilot>());
  if (!simulation.Initialize(opts::simulation::AircraftName, SimHz)) {
    throw std::runtime_error("TECS simulation initialization failed");
  }
  sim::InitialCondition initial = simulation.GetDefaultInitialCondition();
  initial.calibratedAirspeedMps = math::KnotsToMetersPerSecond(80.0);
  if (!simulation.Reset(initial)) {
    throw std::runtime_error("TECS simulation reset failed");
  }
  const gnc::TrimResult *trim = simulation.GetTrimService().GetResult();
  if (trim == nullptr || !trim->success) {
    throw std::runtime_error("TECS simulation trim failed");
  }

  auto &manager = Manager(simulation);
  manager.GetManualController().SetCommandedInput({
      .elevator = trim->elevator,
      .aileron = trim->aileron,
      .rudder = trim->rudder,
      .throttle = trim->throttle,
  });
  auto &autopilot = dynamic_cast<gnc::PX4Autopilot &>(manager.GetAutopilot());
  auto &properties = simulation.GetAircraft().GetProperties();
  const double initialAltitudeM = properties.AltitudeAgl().M();
  const double initialAirspeedMps = properties.CalibratedAirspeed().Mps();
  const double targetAltitudeM = initialAltitudeM + scenario.altitudeDeltaM;
  const double targetAirspeedMps =
      initialAirspeedMps + scenario.airspeedDeltaMps;

  gnc::Px4TecsSettings settings = autopilot.GetTecsSettings();
  ApplyTuningOverrides(settings, overrides);
  autopilot.SetTecsSettings(settings);
  autopilot.SetTargetAltitudeM(initialAltitudeM);
  autopilot.SetTargetAirspeedMps(initialAirspeedMps);
  autopilot.SetTargetRollRad(properties.Roll().Rad());
  autopilot.SetRollHoldEnabled(true);
  autopilot.SetTecsEnabled(true);
  manager.SetMode(control::FlightControlMode::Autopilot);

  std::vector<Sample> samples;
  samples.reserve(static_cast<std::size_t>(RunDurationSec * SimHz));
  bool commandApplied = false;
  for (int index = 0; index < std::lround(RunDurationSec * SimHz);
      ++index) {
    const double timeSec = static_cast<double>(index) / SimHz;
    if (timeSec >= CommandTimeSec && !commandApplied) {
      if (scenario.kind == ScenarioKind::Underspeed) {
        gnc::Px4TecsSettings protectionSettings = autopilot.GetTecsSettings();
        protectionSettings.minimumAirspeedMps = initialAirspeedMps + 2.0;
        protectionSettings.maximumAirspeedMps = initialAirspeedMps + 20.0;
        autopilot.SetTecsSettings(protectionSettings);
      }
      autopilot.SetTargetAltitudeM(targetAltitudeM);
      autopilot.SetTargetAirspeedMps(
          scenario.kind == ScenarioKind::Underspeed
              ? autopilot.GetTecsSettings().minimumAirspeedMps
              : targetAirspeedMps);
      commandApplied = true;
    }
    if (!simulation.Tick()) {
      throw std::runtime_error("TECS simulation tick failed");
    }
    const gnc::Px4TecsDiagnostics &diagnostics = autopilot.GetTecsDiagnostics();
    samples.push_back({
        .timeSec = simulation.GetTime(),
        .altitudeM = properties.AltitudeAgl().M(),
        .airspeedMps = properties.CalibratedAirspeed().Mps(),
        .pitchDeg = properties.Pitch().Deg(),
        .targetPitchDeg = math::RadToDeg(diagnostics.targetPitchRad),
        .throttle = simulation.GetAircraft().GetControls().GetInput().throttle,
        .elevator = simulation.GetAircraft().GetControls().GetInput().elevator,
        .underspeed = diagnostics.underspeedProtectionActive,
        .pitchLimited =
            diagnostics.pitchUpperLimited || diagnostics.pitchLowerLimited,
        .throttleSaturated =
            diagnostics.throttleUpperSaturated
            || diagnostics.throttleLowerSaturated
            || diagnostics.targetThrottle
                   <= autopilot.GetTecsSettings().minimumThrottle + 0.001
            || diagnostics.targetThrottle
                   >= autopilot.GetTecsSettings().maximumThrottle - 0.001,
    });
  }

  const double evaluatedAirspeedTarget =
      scenario.kind == ScenarioKind::Underspeed
          ? autopilot.GetTecsSettings().minimumAirspeedMps
          : targetAirspeedMps;
  return Evaluate(samples,
      targetAltitudeM,
      evaluatedAirspeedTarget,
      initialAltitudeM,
      scenario);
}

bool IsAcceptable(const ScenarioDefinition &scenario, const Metrics &metrics) {
  const bool stable =
      metrics.tailPitchRangeDeg <= MaximumTailPitchRangeDeg
      && metrics.tailThrottleRange <= MaximumTailThrottleRange
      && metrics.tailPitchTrackingErrorDeg <= MaximumPitchTrackingErrorDeg
      && metrics.tailElevatorSaturationDurationSec
             <= MaximumTailSaturationDurationSec
      && metrics.tailThrottleSaturationDurationSec
             <= MaximumTailSaturationDurationSec
      && metrics.tailPitchLimitDurationSec <= MaximumTailSaturationDurationSec;
  if (scenario.kind == ScenarioKind::Underspeed) {
    const double pitchRelaxationDeg = metrics.preCommandTargetPitchDeg
                                      - metrics.minimumProtectedTargetPitchDeg;
    return stable && metrics.underspeedObserved
           && metrics.maximumProtectedThrottle >= 0.95
           && pitchRelaxationDeg >= 0.5 && metrics.minimumAirspeedMps >= 38.0;
  }
  const bool safeAirspeed =
      metrics.minimumAirspeedMps >= 25.0 && metrics.maximumAirspeedMps <= 62.0;
  switch (scenario.kind) {
  case ScenarioKind::Level:
    return stable && metrics.tailAltitudeMaximumErrorM <= 1.0
           && metrics.tailAirspeedMaximumErrorMps <= 0.3;
  case ScenarioKind::Climb:
  case ScenarioKind::Descent:
    return stable && safeAirspeed && metrics.tailAltitudeMaximumErrorM <= 2.0
           && metrics.tailAirspeedMaximumErrorMps <= 0.5
           && metrics.altitudeOvershootM <= 5.0;
  case ScenarioKind::AirspeedIncrease:
  case ScenarioKind::AirspeedDecrease:
    return stable && safeAirspeed && metrics.tailAirspeedMaximumErrorMps <= 0.3
           && metrics.tailAltitudeMaximumErrorM <= 2.0;
  case ScenarioKind::Combined:
    return stable && safeAirspeed && metrics.tailAltitudeMaximumErrorM <= 2.0
           && metrics.tailAirspeedMaximumErrorMps <= 0.3;
  case ScenarioKind::Underspeed:
    break;
  }
  return false;
}

std::string_view ScenarioKey(ScenarioKind kind) {
  switch (kind) {
  case ScenarioKind::Level:
    return "level";
  case ScenarioKind::Climb:
    return "climb";
  case ScenarioKind::Descent:
    return "descent";
  case ScenarioKind::AirspeedIncrease:
    return "airspeed-up";
  case ScenarioKind::AirspeedDecrease:
    return "airspeed-down";
  case ScenarioKind::Combined:
    return "combined";
  case ScenarioKind::Underspeed:
    return "underspeed";
  }
  return "unknown";
}

double ParseNumber(std::string_view option, const char *value) {
  try {
    std::size_t parsed = 0;
    const std::string text(value);
    const double result = std::stod(text, &parsed);
    if (parsed != text.size() || !std::isfinite(result)) {
      throw std::invalid_argument("not a finite number");
    }
    return result;
  } catch (const std::exception &) {
    throw std::invalid_argument(
        "Invalid value for " + std::string(option) + ": " + value);
  }
}

Options ParseOptions(int argc, char **argv) {
  Options options;
  const auto requireValue = [&](int &index, std::string_view option) {
    if (++index >= argc) {
      throw std::invalid_argument("Missing value for " + std::string(option));
    }
    return argv[index];
  };
  const auto assignNumber =
      [&](std::optional<double> &target, int &index, std::string_view option) {
        target = ParseNumber(option, requireValue(index, option));
      };

  for (int index = 1; index < argc; ++index) {
    const std::string_view option(argv[index]);
    if (option == "--scenario") {
      const std::string_view value(requireValue(index, option));
      constexpr ScenarioKind kinds[]{ScenarioKind::Level,
          ScenarioKind::Climb,
          ScenarioKind::Descent,
          ScenarioKind::AirspeedIncrease,
          ScenarioKind::AirspeedDecrease,
          ScenarioKind::Combined,
          ScenarioKind::Underspeed};
      for (const ScenarioKind kind : kinds) {
        if (value == ScenarioKey(kind)) {
          options.scenario = kind;
          break;
        }
      }
      if (!options.scenario) {
        throw std::invalid_argument("Unknown scenario: " + std::string(value));
      }
    } else if (option == "--altitude-gain") {
      assignNumber(options.tuning.altitudeErrorGain, index, option);
    } else if (option == "--airspeed-gain") {
      assignNumber(options.tuning.airspeedErrorGain, index, option);
    } else if (option == "--throttle-damping") {
      assignNumber(options.tuning.throttleDampingGain, index, option);
    } else if (option == "--throttle-integral") {
      assignNumber(options.tuning.throttleIntegralGain, index, option);
    } else if (option == "--pitch-damping") {
      assignNumber(options.tuning.pitchDampingGain, index, option);
    } else if (option == "--pitch-integral") {
      assignNumber(options.tuning.pitchIntegralGain, index, option);
    } else if (option == "--seb-feed-forward") {
      assignNumber(options.tuning.energyBalanceFeedForwardGain, index, option);
    } else if (option == "--energy-rate-time-constant") {
      assignNumber(options.tuning.totalEnergyRateFilterTimeConstantSec,
          index,
          option);
    } else if (option == "--pitch-slew-deg-s") {
      assignNumber(options.tuning.pitchSlewRateDegPerSec, index, option);
    } else if (option == "--throttle-slew") {
      assignNumber(options.tuning.throttleSlewRatePerSec, index, option);
    } else if (option == "--climb-rate") {
      assignNumber(options.tuning.maximumClimbRateMps, index, option);
    } else if (option == "--sink-rate") {
      assignNumber(options.tuning.maximumSinkRateMps, index, option);
    } else {
      throw std::invalid_argument("Unknown option: " + std::string(option));
    }
  }
  return options;
}

void PrintMetrics(const ScenarioDefinition &scenario, const Metrics &metrics,
    bool accepted) {
  const double pitchRelaxationDeg =
      metrics.underspeedObserved ? metrics.preCommandTargetPitchDeg
                                       - metrics.minimumProtectedTargetPitchDeg
                                 : 0.0;
  std::cout
      << "scenario=" << ScenarioKey(scenario.kind)
      << ", altitude_mean_error_m=" << metrics.tailAltitudeMeanErrorM
      << ", altitude_max_error_m=" << metrics.tailAltitudeMaximumErrorM
      << ", altitude_rms_error_m=" << metrics.tailAltitudeRmsErrorM
      << ", airspeed_mean_error_mps=" << metrics.tailAirspeedMeanErrorMps
      << ", airspeed_max_error_mps=" << metrics.tailAirspeedMaximumErrorMps
      << ", airspeed_rms_error_mps=" << metrics.tailAirspeedRmsErrorMps
      << ", altitude_overshoot_m=" << metrics.altitudeOvershootM
      << ", settling_time_s=" << metrics.settlingTimeSec
      << ", airspeed_min_mps=" << metrics.minimumAirspeedMps
      << ", airspeed_max_mps=" << metrics.maximumAirspeedMps
      << ", pitch_min_deg=" << metrics.minimumPitchDeg
      << ", pitch_max_deg=" << metrics.maximumPitchDeg
      << ", target_pitch_min_deg=" << metrics.minimumTargetPitchDeg
      << ", throttle_min=" << metrics.minimumThrottle
      << ", throttle_max=" << metrics.maximumThrottle
      << ", altitude_tail_range_m=" << metrics.tailAltitudeRangeM
      << ", airspeed_tail_range_mps=" << metrics.tailAirspeedRangeMps
      << ", pitch_tail_range_deg=" << metrics.tailPitchRangeDeg
      << ", throttle_tail_range=" << metrics.tailThrottleRange
      << ", pitch_tracking_error_deg=" << metrics.tailPitchTrackingErrorDeg
      << ", elevator_saturation_s=" << metrics.elevatorSaturationDurationSec
      << ", throttle_saturation_s=" << metrics.throttleSaturationDurationSec
      << ", tail_elevator_saturation_s="
      << metrics.tailElevatorSaturationDurationSec
      << ", tail_throttle_saturation_s="
      << metrics.tailThrottleSaturationDurationSec
      << ", tail_pitch_limit_s=" << metrics.tailPitchLimitDurationSec
      << ", underspeed=" << (metrics.underspeedObserved ? 1 : 0)
      << ", underspeed_activation_s=" << metrics.underspeedActivationTimeSec
      << ", protected_max_throttle=" << metrics.maximumProtectedThrottle
      << ", pitch_relaxation_deg=" << pitchRelaxationDeg
      << ", result=" << (accepted ? "PASS" : "FAIL") << '\n';
}
} // namespace

int main(int argc, char **argv) {
  const std::vector<ScenarioDefinition> scenarios{
      {"Level Hold", ScenarioKind::Level, 0.0, 0.0},
      {"Altitude +50 m", ScenarioKind::Climb, 50.0, 0.0},
      {"Altitude -50 m", ScenarioKind::Descent, -50.0, 0.0},
      {"Airspeed +5 m/s", ScenarioKind::AirspeedIncrease, 0.0, 5.0},
      {"Airspeed -5 m/s", ScenarioKind::AirspeedDecrease, 0.0, -5.0},
      {"Combined +50 m/+3 m/s", ScenarioKind::Combined, 50.0, 3.0},
      {"Underspeed protection", ScenarioKind::Underspeed, 50.0, 0.0},
  };

  try {
    const Options options = ParseOptions(argc, argv);
    bool allAccepted = true;
    bool scenarioExecuted = false;
    std::cout << std::fixed << std::setprecision(3);
    for (const ScenarioDefinition &scenario : scenarios) {
      if (options.scenario && *options.scenario != scenario.kind) {
        continue;
      }
      scenarioExecuted = true;
      const Metrics metrics = Execute(scenario, options.tuning);
      const bool accepted = IsAcceptable(scenario, metrics);
      allAccepted = allAccepted && accepted;
      PrintMetrics(scenario, metrics, accepted);
    }
    return scenarioExecuted && allAccepted ? EXIT_SUCCESS : EXIT_FAILURE;
  } catch (const std::exception &error) {
    std::cerr << "px4_tecs_integration_tests: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
