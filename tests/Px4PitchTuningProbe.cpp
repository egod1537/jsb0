#include "common/math/Math.hpp"
#include "sim/InitialCondition.hpp"
#include "sim/Simulation.hpp"
#include "sim/control/FlightControlManager.hpp"
#include "sim/control/FlightControlMode.hpp"
#include "sim/gnc/TrimTypes.hpp"
#include "sim/gnc/autopilot/PX4Autopilot.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
constexpr double StepTimeSec = 3.0;
constexpr double RunDurationSec = 15.0;
constexpr double SettlingBandDeg = 0.5;

struct Candidate {
  double timeConstantSec{};
  double p{};
  double i{};
  double d{};
  double ff{};
  double maximumPositiveRateDegPerSec = 60.0;
  double maximumNegativeRateDegPerSec = 60.0;
  double integratorLimit = 0.4;
};

struct RollCandidate {
  double timeConstantSec = 0.4;
  double p = 1.9;
  double i = 0.25;
  double d = 0.0;
  double ff = 1.2;
};

struct Sample {
  double timeSec{};
  double relativePitchDeg{};
  double qDegPerSec{};
  double elevator{};
  bool saturated{};
};

struct Metrics {
  double riseTimeSec{std::numeric_limits<double>::infinity()};
  double settlingSec{std::numeric_limits<double>::infinity()};
  double overshootDeg{};
  double steadyErrorDeg{};
  double tailRangeDeg{};
  double rmsPitchRateDegPerSec{};
  double maxElevatorDelta{};
  double saturationFraction{};
  double minimumAirspeedKts{std::numeric_limits<double>::infinity()};
  double score{};
  bool acceptable{};
};

struct Run {
  std::string scenario;
  double hz{};
  double airspeedKts{};
  double stepDeg{};
  Candidate candidate;
  Metrics metrics;
};

struct RankedCandidate {
  Candidate candidate;
  Metrics nominal;
};

double Mean(const std::vector<double> &values) {
  return values.empty() ? 0.0
                        : std::accumulate(values.begin(), values.end(), 0.0)
                              / static_cast<double>(values.size());
}

double Rms(const std::vector<double> &values) {
  if (values.empty()) {
    return 0.0;
  }
  return std::sqrt(
      std::accumulate(values.begin(),
          values.end(),
          0.0,
          [](double sum, double value) { return sum + value * value; })
      / static_cast<double>(values.size()));
}

double FiniteOr(double value, double fallback) {
  return std::isfinite(value) ? value : fallback;
}

control::FlightControlManager &Manager(sim::Simulation &simulation) {
  return simulation.GetFlightControlManager();
}

gnc::PX4Autopilot &Autopilot(sim::Simulation &simulation) {
  return dynamic_cast<gnc::PX4Autopilot &>(Manager(simulation).GetAutopilot());
}

Metrics Evaluate(const std::vector<Sample> &samples, double stepDeg,
    double trimElevator, double minimumAirspeedKts) {
  Metrics metrics;
  metrics.minimumAirspeedKts = minimumAirspeedKts;
  const double direction = std::copysign(1.0, stepDeg);
  const double targetMagnitude = std::abs(stepDeg);
  const double riseThreshold = targetMagnitude * 0.9;
  double maximumDirectedPitch = -std::numeric_limits<double>::infinity();
  double lastOutsideTimeSec = StepTimeSec;
  bool outsideAtEnd = true;
  std::size_t saturatedSamples = 0;
  std::vector<double> tailPitch;
  std::vector<double> tailErrors;
  std::vector<double> tailPitchRates;

  for (const Sample &sample : samples) {
    saturatedSamples += sample.saturated ? 1u : 0u;
    metrics.maxElevatorDelta = std::max(metrics.maxElevatorDelta,
        std::abs(sample.elevator - trimElevator));
    if (sample.timeSec < StepTimeSec) {
      continue;
    }

    const double directedPitch = direction * sample.relativePitchDeg;
    maximumDirectedPitch = std::max(maximumDirectedPitch, directedPitch);
    if (!std::isfinite(metrics.riseTimeSec) && directedPitch >= riseThreshold) {
      metrics.riseTimeSec = sample.timeSec - StepTimeSec;
    }
    const double errorDeg = stepDeg - sample.relativePitchDeg;
    outsideAtEnd = std::abs(errorDeg) > SettlingBandDeg;
    if (outsideAtEnd) {
      lastOutsideTimeSec = sample.timeSec;
    }
    if (sample.timeSec >= RunDurationSec - 3.0) {
      tailPitch.push_back(sample.relativePitchDeg);
      tailErrors.push_back(errorDeg);
      tailPitchRates.push_back(sample.qDegPerSec);
    }
  }

  if (!outsideAtEnd) {
    metrics.settlingSec = std::max(0.0, lastOutsideTimeSec - StepTimeSec);
  }
  metrics.overshootDeg = std::max(0.0, maximumDirectedPitch - targetMagnitude);
  metrics.steadyErrorDeg = Mean(tailErrors);
  if (!tailPitch.empty()) {
    const auto [minimum, maximum] =
        std::minmax_element(tailPitch.begin(), tailPitch.end());
    metrics.tailRangeDeg = *maximum - *minimum;
  }
  metrics.rmsPitchRateDegPerSec = Rms(tailPitchRates);
  metrics.saturationFraction =
      static_cast<double>(saturatedSamples) / samples.size();

  const auto cost = [](double value, double scale) {
    return std::min(std::abs(value) / scale, 10.0);
  };
  const std::array costs{
      cost(FiniteOr(metrics.riseTimeSec, 12.0), 2.5),
      cost(FiniteOr(metrics.settlingSec, 12.0), 5.0),
      cost(metrics.overshootDeg, 1.0),
      cost(metrics.steadyErrorDeg, 0.25),
      cost(metrics.tailRangeDeg, 0.5),
      cost(metrics.rmsPitchRateDegPerSec, 0.5),
      cost(metrics.maxElevatorDelta, 0.35),
      cost(metrics.saturationFraction, 0.01),
  };
  metrics.score =
      std::accumulate(costs.begin(), costs.end(), 0.0) / costs.size();
  metrics.acceptable =
      metrics.riseTimeSec <= 4.0 && metrics.settlingSec <= 7.0
      && metrics.overshootDeg <= 1.5 && std::abs(metrics.steadyErrorDeg) <= 0.35
      && metrics.tailRangeDeg <= 0.75 && metrics.maxElevatorDelta <= 0.5
      && metrics.saturationFraction == 0.0;
  return metrics;
}

Run Execute(std::string scenario, double hz, double airspeedKts, double stepDeg,
    Candidate candidate) {
  sim::Simulation simulation(std::make_unique<gnc::PX4Autopilot>());
  if (!simulation.Initialize(opts::simulation::AircraftName, hz)) {
    throw std::runtime_error("Failed to initialize pitch tuning simulation");
  }
  sim::InitialCondition initial = simulation.GetDefaultInitialCondition();
  initial.calibratedAirspeedMps =
      math::KnotsToMetersPerSecond(airspeedKts);
  if (!simulation.Reset(initial)) {
    throw std::runtime_error("Failed to reset pitch tuning initial state");
  }

  const gnc::TrimResult *trim = simulation.GetTrimService().GetResult();
  if (trim == nullptr || !trim->success) {
    throw std::runtime_error("Pitch tuning trim failed");
  }
  Manager(simulation)
      .GetManualController()
      .SetCommandedInput({
          .elevator = trim->elevator,
          .aileron = trim->aileron,
          .rudder = trim->rudder,
          .throttle = trim->throttle,
      });

  auto &autopilot = Autopilot(simulation);
  auto pitchSettings = autopilot.GetPitchHoldSettings();
  pitchSettings.timeConstantSec = candidate.timeConstantSec;
  pitchSettings.rateProportionalGain = candidate.p;
  pitchSettings.rateIntegralGain = candidate.i;
  pitchSettings.rateDerivativeGain = candidate.d;
  pitchSettings.rateFeedForwardGain = candidate.ff;
  pitchSettings.maximumPositivePitchRateRadPerSec =
      math::DegToRad(candidate.maximumPositiveRateDegPerSec);
  pitchSettings.maximumNegativePitchRateRadPerSec =
      math::DegToRad(candidate.maximumNegativeRateDegPerSec);
  pitchSettings.integratorLimit = candidate.integratorLimit;
  autopilot.SetPitchHoldSettings(pitchSettings);

  const double initialPitchRad =
      simulation.GetAircraft().GetProperties().Pitch().Rad();
  autopilot.SetTargetPitchRad(initialPitchRad);
  const double targetRollRad =
      simulation.GetAircraft().GetProperties().Roll().Rad();
  autopilot.SetTargetRollRad(targetRollRad);
  autopilot.SetPitchHoldEnabled(true);
  autopilot.SetRollHoldEnabled(true);
  Manager(simulation).SetMode(control::FlightControlMode::Autopilot);

  std::vector<Sample> samples;
  samples.reserve(static_cast<std::size_t>(RunDurationSec * hz));
  double minimumAirspeed = std::numeric_limits<double>::infinity();
  for (int tick = 0; tick < std::lround(RunDurationSec * hz); ++tick) {
    const double timeSec = static_cast<double>(tick) / hz;
    if (timeSec >= StepTimeSec) {
      autopilot.SetTargetPitchRad(initialPitchRad + math::DegToRad(stepDeg));
    }
    if (!simulation.Tick()) {
      throw std::runtime_error("Pitch tuning simulation tick failed");
    }
    const auto &diagnostics = autopilot.GetPitchHoldDiagnostics();
    const auto &properties = simulation.GetAircraft().GetProperties();
    minimumAirspeed =
        std::min(minimumAirspeed, properties.CalibratedAirspeed().Kts());
    samples.push_back({simulation.GetTime(),
        properties.Pitch().Deg() - math::RadToDeg(initialPitchRad),
        properties.Q().DegPerSec(),
        diagnostics.elevatorCommand,
        diagnostics.positiveSaturation || diagnostics.negativeSaturation});
  }

  return {std::move(scenario),
      hz,
      airspeedKts,
      stepDeg,
      candidate,
      Evaluate(samples, stepDeg, trim->elevator, minimumAirspeed)};
}

bool SameCandidate(const Candidate &left, const Candidate &right) {
  return left.timeConstantSec == right.timeConstantSec && left.p == right.p
         && left.i == right.i && left.d == right.d && left.ff == right.ff
         && left.maximumPositiveRateDegPerSec
                == right.maximumPositiveRateDegPerSec
         && left.maximumNegativeRateDegPerSec
                == right.maximumNegativeRateDegPerSec
         && left.integratorLimit == right.integratorLimit;
}

void WriteRuns(const std::filesystem::path &path,
    const std::vector<Run> &runs) {
  std::ofstream output(path);
  output << "scenario,hz,airspeed_kts,step_deg,tc,p,i,d,ff,rise_sec,"
            "settling_sec,overshoot_deg,steady_error_deg,tail_range_deg,"
            "rms_q_deg_s,max_elevator_delta,saturation_fraction,"
            "minimum_airspeed_kts,score,acceptable\n";
  for (const Run &run : runs) {
    const Metrics &m = run.metrics;
    output << run.scenario << ',' << run.hz << ',' << run.airspeedKts << ','
           << run.stepDeg << ',' << run.candidate.timeConstantSec << ','
           << run.candidate.p << ',' << run.candidate.i << ','
           << run.candidate.d << ',' << run.candidate.ff << ','
           << FiniteOr(m.riseTimeSec, -1.0) << ','
           << FiniteOr(m.settlingSec, -1.0) << ',' << m.overshootDeg << ','
           << m.steadyErrorDeg << ',' << m.tailRangeDeg << ','
           << m.rmsPitchRateDegPerSec << ',' << m.maxElevatorDelta << ','
           << m.saturationFraction << ',' << m.minimumAirspeedKts << ','
           << m.score << ',' << (m.acceptable ? 1 : 0) << '\n';
  }
}

std::vector<RankedCandidate> SweepNominal() {
  constexpr std::array TimeConstants{0.2, 0.3, 0.4, 0.5};
  constexpr std::array ProportionalGains{0.3, 0.6, 1.0, 1.5};
  constexpr std::array IntegralGains{0.2, 0.35, 0.5, 0.8};
  constexpr std::array DerivativeGains{0.0};
  constexpr std::array FeedForwardGains{1.2, 1.8, 2.4};
  std::vector<RankedCandidate> ranked;
  for (double timeConstant : TimeConstants) {
    for (double p : ProportionalGains) {
      for (double i : IntegralGains) {
        for (double d : DerivativeGains) {
          for (double ff : FeedForwardGains) {
            const Candidate candidate{timeConstant, p, i, d, ff};
            const Run run = Execute("nominal", 120.0, 80.0, 3.0, candidate);
            ranked.push_back({candidate, run.metrics});
          }
        }
      }
    }
  }
  std::ranges::sort(ranked, {}, [](const RankedCandidate &entry) {
    return entry.nominal.score;
  });
  return ranked;
}

std::vector<Run> ValidateCandidates(
    const std::vector<RankedCandidate> &ranked) {
  constexpr std::array Frequencies{30.0, 120.0, 240.0};
  constexpr std::array Airspeeds{60.0, 80.0, 110.0};
  constexpr std::array Steps{-3.0, 3.0};
  std::vector<Candidate> candidates{
      {0.4, 0.08, 0.1, 0.0, 0.5},
      {0.3, 0.6, 0.8, 0.0, 1.2},
  };
  for (const RankedCandidate &entry : ranked) {
    if (candidates.size() >= 9) {
      break;
    }
    if (std::ranges::none_of(candidates, [&entry](const Candidate &candidate) {
          return SameCandidate(candidate, entry.candidate);
        })) {
      candidates.push_back(entry.candidate);
    }
  }

  std::vector<Run> runs;
  for (std::size_t candidateIndex = 0; candidateIndex < candidates.size();
      ++candidateIndex) {
    for (double hz : Frequencies) {
      for (double airspeed : Airspeeds) {
        for (double step : Steps) {
          runs.push_back(Execute("candidate_" + std::to_string(candidateIndex),
              hz,
              airspeed,
              step,
              candidates[candidateIndex]));
        }
      }
    }
  }
  return runs;
}

void PrintSummary(const std::vector<RankedCandidate> &ranked,
    const std::vector<Run> &validation) {
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "Top nominal candidates:\n";
  for (std::size_t index = 0; index < std::min<std::size_t>(8, ranked.size());
      ++index) {
    const auto &entry = ranked[index];
    std::cout << index << ": tc=" << entry.candidate.timeConstantSec
              << " p=" << entry.candidate.p << " i=" << entry.candidate.i
              << " d=" << entry.candidate.d << " ff=" << entry.candidate.ff
              << " score=" << entry.nominal.score
              << " settling=" << entry.nominal.settlingSec
              << " overshoot=" << entry.nominal.overshootDeg
              << " steady=" << entry.nominal.steadyErrorDeg << '\n';
  }

  std::cout << "\nRobust validation aggregate:\n";
  for (int candidateIndex = 0; candidateIndex < 9; ++candidateIndex) {
    std::vector<const Run *> selected;
    const std::string scenario = "candidate_" + std::to_string(candidateIndex);
    for (const Run &run : validation) {
      if (run.scenario == scenario) {
        selected.push_back(&run);
      }
    }
    if (selected.empty()) {
      continue;
    }
    double meanScore = 0.0;
    double worstScore = 0.0;
    int acceptable = 0;
    for (const Run *run : selected) {
      meanScore += run->metrics.score;
      worstScore = std::max(worstScore, run->metrics.score);
      acceptable += run->metrics.acceptable ? 1 : 0;
    }
    meanScore /= selected.size();
    const Candidate &candidate = selected.front()->candidate;
    std::cout << candidateIndex << ": tc=" << candidate.timeConstantSec
              << " p=" << candidate.p << " i=" << candidate.i
              << " d=" << candidate.d << " ff=" << candidate.ff
              << " mean_score=" << meanScore << " worst=" << worstScore
              << " acceptable=" << acceptable << '/' << selected.size() << '\n';
  }
}

struct TransitionDefinition {
  std::string_view name;
  double sourcePitchDeg{};
  double targetPitchDeg{};
};

struct TransitionMetrics {
  double overshootDeg{};
  double settlingSec{std::numeric_limits<double>::infinity()};
  double steadyMaxPitchErrorDeg{};
  double steadyRmsPitchErrorDeg{};
  double steadyMaxPitchRateDegPerSec{};
  double steadyRmsPitchRateDegPerSec{};
  double steadyMaxPitchRateErrorDegPerSec{};
  double steadyMaxRollErrorDeg{};
  double maximumRollErrorDeg{};
  double maximumRollRateDegPerSec{};
  double maximumYawRateDegPerSec{};
  double maximumSideslipDeg{};
  double maximumAileronMagnitude{};
  double maximumRollIntegratorMagnitude{};
  double rollSaturationDurationSec{};
  std::size_t rollSaturationCount{};
  double rollIntegratorLimitedDurationSec{};
  std::size_t rollIntegratorLimitedCount{};
  double steadyElevatorRange{};
  double maximumElevator{};
  double minimumElevator{};
  double saturationDurationSec{};
  std::size_t saturationCount{};
  double steadySaturationDurationSec{};
  double integratorLimitedDurationSec{};
  std::size_t integratorLimitedCount{};
  double steadyIntegratorLimitedDurationSec{};
  double maximumIntegratorMagnitude{};
  double finalIntegrator{};
  double minimumAirspeedKts{std::numeric_limits<double>::infinity()};
  double maximumAirspeedKts{};
  bool sourceConditionValid{};
  bool requiredPass{};
  bool qualityPass{};
};

struct TransitionSample;

struct TransitionRun {
  TransitionDefinition definition;
  Candidate candidate;
  RollCandidate rollCandidate;
  double hz{};
  double initialAirspeedKts{};
  bool rollHoldEnabled{};
  bool yawRateControlEnabled{};
  TransitionMetrics metrics;
  std::vector<TransitionSample> preconditionSamples;
  std::vector<TransitionSample> responseSamples;
};

struct TransitionSample {
  double timeSec{};
  double pitchDeg{};
  double pitchErrorDeg{};
  double pitchRateSetpointDegPerSec{};
  double pitchRateDegPerSec{};
  double pitchRateErrorDegPerSec{};
  double rollDeg{};
  double rollRateDegPerSec{};
  double yawRateDegPerSec{};
  double sideslipDeg{};
  double elevator{};
  double aileron{};
  double rudder{};
  double rollRateSetpointDegPerSec{};
  double rollRateErrorDegPerSec{};
  double rollProportionalTerm{};
  double rollIntegralTerm{};
  double rollDerivativeTerm{};
  double rollFeedForwardTerm{};
  double rollIntegrator{};
  double proportionalTerm{};
  double integralTerm{};
  double derivativeTerm{};
  double feedForwardTerm{};
  double integrator{};
  double airspeedKts{};
  bool saturated{};
  bool integratorLimited{};
  bool rollSaturated{};
  bool rollIntegratorLimited{};
};

constexpr double TransitionPreconditionSec = 15.0;
constexpr double TransitionObservationSec = 15.0;
constexpr double SteadyWindowSec = 2.0;
constexpr double RequiredPitchErrorDeg = 0.1;
constexpr double RequiredPitchRateDegPerSec = 0.1;
constexpr double QualitySettlingBandDeg = 0.2;
constexpr double QualitySettlingLimitSec = 3.0;
constexpr double QualityOvershootDeg = 0.5;

TransitionSample CaptureTransitionSample(sim::Simulation &simulation,
    const gnc::PX4Autopilot &autopilot, double timeSec) {
  const auto &properties = simulation.GetAircraft().GetProperties();
  const auto &controls = simulation.GetAircraft().GetControls().GetInput();
  const auto &diagnostics = autopilot.GetPitchHoldDiagnostics();
  const auto &rollDiagnostics = autopilot.GetRollHoldDiagnostics();
  return {
      .timeSec = timeSec,
      .pitchDeg = properties.Pitch().Deg(),
      .pitchErrorDeg = math::RadToDeg(diagnostics.pitchErrorRad),
      .pitchRateSetpointDegPerSec =
          math::RadToDeg(diagnostics.bodyRateSetpointRadPerSec),
      .pitchRateDegPerSec = properties.Q().DegPerSec(),
      .pitchRateErrorDegPerSec =
          math::RadToDeg(diagnostics.bodyRateErrorRadPerSec),
      .rollDeg = properties.Roll().Deg(),
      .rollRateDegPerSec = properties.P().DegPerSec(),
      .yawRateDegPerSec = properties.R().DegPerSec(),
      .sideslipDeg = properties.Beta().Deg(),
      .elevator = diagnostics.elevatorCommand,
      .aileron = controls.aileron,
      .rudder = controls.rudder,
      .rollRateSetpointDegPerSec =
          math::RadToDeg(rollDiagnostics.bodyRateSetpointRadPerSec),
      .rollRateErrorDegPerSec =
          math::RadToDeg(rollDiagnostics.bodyRateErrorRadPerSec),
      .rollProportionalTerm = rollDiagnostics.rateProportionalTerm,
      .rollIntegralTerm = rollDiagnostics.rateIntegralTerm,
      .rollDerivativeTerm = rollDiagnostics.rateDerivativeTerm,
      .rollFeedForwardTerm = rollDiagnostics.rateFeedForwardTerm,
      .rollIntegrator = rollDiagnostics.rateIntegrator,
      .proportionalTerm = diagnostics.rateProportionalTerm,
      .integralTerm = diagnostics.rateIntegralTerm,
      .derivativeTerm = diagnostics.rateDerivativeTerm,
      .feedForwardTerm = diagnostics.rateFeedForwardTerm,
      .integrator = diagnostics.rateIntegrator,
      .airspeedKts = properties.CalibratedAirspeed().Kts(),
      .saturated =
          diagnostics.positiveSaturation || diagnostics.negativeSaturation,
      .integratorLimited = diagnostics.integratorLimited,
      .rollSaturated = rollDiagnostics.positiveSaturation
                       || rollDiagnostics.negativeSaturation,
      .rollIntegratorLimited = rollDiagnostics.integratorLimited,
  };
}

TransitionMetrics EvaluateTransition(
    std::span<const TransitionSample> preconditionSamples,
    std::span<const TransitionSample> responseSamples,
    const TransitionDefinition &definition, double targetRollDeg, double hz) {
  TransitionMetrics metrics;
  const double direction =
      std::copysign(1.0, definition.targetPitchDeg - definition.sourcePitchDeg);
  const std::size_t steadySampleCount =
      static_cast<std::size_t>(std::lround(SteadyWindowSec * hz));
  const std::size_t steadyStart =
      responseSamples.size() > steadySampleCount
          ? responseSamples.size() - steadySampleCount
          : 0;
  const std::size_t sourceStart =
      preconditionSamples.size() > steadySampleCount
          ? preconditionSamples.size() - steadySampleCount
          : 0;
  double lastOutsideTimeSec = 0.0;
  bool outsideAtEnd = true;
  bool previouslySaturated = false;
  bool previouslyLimited = false;
  bool rollPreviouslySaturated = false;
  bool rollPreviouslyLimited = false;
  std::vector<double> steadyErrors;
  std::vector<double> steadyRates;
  std::vector<double> steadyElevators;

  metrics.maximumElevator = -std::numeric_limits<double>::infinity();
  metrics.minimumElevator = std::numeric_limits<double>::infinity();
  metrics.sourceConditionValid = true;
  for (std::size_t index = sourceStart; index < preconditionSamples.size();
      ++index) {
    const TransitionSample &sample = preconditionSamples[index];
    metrics.sourceConditionValid =
        metrics.sourceConditionValid
        && std::abs(definition.sourcePitchDeg - sample.pitchDeg)
               <= RequiredPitchErrorDeg
        && std::abs(sample.pitchRateDegPerSec) <= RequiredPitchRateDegPerSec
        && !sample.saturated && !sample.integratorLimited;
  }

  for (std::size_t index = 0; index < responseSamples.size(); ++index) {
    const TransitionSample &sample = responseSamples[index];
    const double directedOvershoot =
        direction * (sample.pitchDeg - definition.targetPitchDeg);
    metrics.overshootDeg = std::max(metrics.overshootDeg, directedOvershoot);
    outsideAtEnd = std::abs(sample.pitchErrorDeg) > QualitySettlingBandDeg;
    if (outsideAtEnd) {
      lastOutsideTimeSec = sample.timeSec;
    }
    metrics.maximumElevator =
        std::max(metrics.maximumElevator, sample.elevator);
    metrics.minimumElevator =
        std::min(metrics.minimumElevator, sample.elevator);
    metrics.maximumIntegratorMagnitude =
        std::max(metrics.maximumIntegratorMagnitude,
            std::abs(sample.integrator));
    metrics.minimumAirspeedKts =
        std::min(metrics.minimumAirspeedKts, sample.airspeedKts);
    metrics.maximumAirspeedKts =
        std::max(metrics.maximumAirspeedKts, sample.airspeedKts);
    metrics.maximumRollErrorDeg = std::max(metrics.maximumRollErrorDeg,
        std::abs(targetRollDeg - sample.rollDeg));
    metrics.maximumRollRateDegPerSec =
        std::max(metrics.maximumRollRateDegPerSec,
            std::abs(sample.rollRateDegPerSec));
    metrics.maximumYawRateDegPerSec = std::max(metrics.maximumYawRateDegPerSec,
        std::abs(sample.yawRateDegPerSec));
    metrics.maximumSideslipDeg =
        std::max(metrics.maximumSideslipDeg, std::abs(sample.sideslipDeg));
    metrics.maximumAileronMagnitude =
        std::max(metrics.maximumAileronMagnitude, std::abs(sample.aileron));
    metrics.maximumRollIntegratorMagnitude =
        std::max(metrics.maximumRollIntegratorMagnitude,
            std::abs(sample.rollIntegrator));
    metrics.rollSaturationDurationSec += sample.rollSaturated ? 1.0 / hz : 0.0;
    metrics.rollIntegratorLimitedDurationSec +=
        sample.rollIntegratorLimited ? 1.0 / hz : 0.0;
    if (sample.rollSaturated && !rollPreviouslySaturated) {
      ++metrics.rollSaturationCount;
    }
    if (sample.rollIntegratorLimited && !rollPreviouslyLimited) {
      ++metrics.rollIntegratorLimitedCount;
    }
    rollPreviouslySaturated = sample.rollSaturated;
    rollPreviouslyLimited = sample.rollIntegratorLimited;
    metrics.saturationDurationSec += sample.saturated ? 1.0 / hz : 0.0;
    metrics.integratorLimitedDurationSec +=
        sample.integratorLimited ? 1.0 / hz : 0.0;
    if (sample.saturated && !previouslySaturated) {
      ++metrics.saturationCount;
    }
    if (sample.integratorLimited && !previouslyLimited) {
      ++metrics.integratorLimitedCount;
    }
    previouslySaturated = sample.saturated;
    previouslyLimited = sample.integratorLimited;

    if (index >= steadyStart) {
      steadyErrors.push_back(sample.pitchErrorDeg);
      steadyRates.push_back(sample.pitchRateDegPerSec);
      steadyElevators.push_back(sample.elevator);
      metrics.steadyMaxPitchErrorDeg = std::max(metrics.steadyMaxPitchErrorDeg,
          std::abs(sample.pitchErrorDeg));
      metrics.steadyMaxPitchRateDegPerSec =
          std::max(metrics.steadyMaxPitchRateDegPerSec,
              std::abs(sample.pitchRateDegPerSec));
      metrics.steadyMaxPitchRateErrorDegPerSec =
          std::max(metrics.steadyMaxPitchRateErrorDegPerSec,
              std::abs(sample.pitchRateErrorDegPerSec));
      metrics.steadyMaxRollErrorDeg = std::max(metrics.steadyMaxRollErrorDeg,
          std::abs(targetRollDeg - sample.rollDeg));
      metrics.steadySaturationDurationSec += sample.saturated ? 1.0 / hz : 0.0;
      metrics.steadyIntegratorLimitedDurationSec +=
          sample.integratorLimited ? 1.0 / hz : 0.0;
    }
  }

  if (!outsideAtEnd) {
    metrics.settlingSec = lastOutsideTimeSec;
  }
  metrics.overshootDeg = std::max(metrics.overshootDeg, 0.0);
  metrics.steadyRmsPitchErrorDeg = Rms(steadyErrors);
  metrics.steadyRmsPitchRateDegPerSec = Rms(steadyRates);
  if (!steadyElevators.empty()) {
    const auto [minimum, maximum] =
        std::minmax_element(steadyElevators.begin(), steadyElevators.end());
    metrics.steadyElevatorRange = *maximum - *minimum;
  }
  metrics.finalIntegrator = responseSamples.back().integrator;
  metrics.requiredPass =
      metrics.sourceConditionValid
      && metrics.steadyMaxPitchErrorDeg <= RequiredPitchErrorDeg
      && metrics.steadyMaxPitchRateDegPerSec <= RequiredPitchRateDegPerSec
      && metrics.steadySaturationDurationSec == 0.0
      && metrics.steadyIntegratorLimitedDurationSec == 0.0;
  metrics.qualityPass = metrics.requiredPass
                        && metrics.overshootDeg <= QualityOvershootDeg
                        && metrics.settlingSec <= QualitySettlingLimitSec;
  return metrics;
}

TransitionRun ExecuteTransition(TransitionDefinition definition,
    Candidate candidate, double hz, double airspeedKts, bool rollHoldEnabled,
    bool yawRateControlEnabled, bool yawRateAugmented,
    const RollCandidate &rollCandidate, double throttlePitchGain) {
  sim::Simulation simulation(std::make_unique<gnc::PX4Autopilot>());
  if (!simulation.Initialize(opts::simulation::AircraftName, hz)) {
    throw std::runtime_error("Failed to initialize transition simulation");
  }
  sim::InitialCondition initial = simulation.GetDefaultInitialCondition();
  initial.altitudeAslM = math::FeetToMeters(3000.0);
  initial.calibratedAirspeedMps =
      math::KnotsToMetersPerSecond(airspeedKts);
  if (!simulation.Reset(initial)) {
    throw std::runtime_error("Failed to reset transition simulation");
  }

  const gnc::TrimResult *trim = simulation.GetTrimService().GetResult();
  if (trim == nullptr || !trim->success) {
    throw std::runtime_error("Transition simulation trim failed");
  }
  auto &manager = Manager(simulation);
  control::ControlInput passthrough{
      .elevator = trim->elevator,
      .aileron = trim->aileron,
      .rudder = trim->rudder,
      .throttle = std::clamp(
          trim->throttle + throttlePitchGain * definition.sourcePitchDeg,
          0.0,
          1.0),
  };
  manager.GetManualController().SetCommandedInput(passthrough);

  auto &autopilot = Autopilot(simulation);
  auto settings = autopilot.GetPitchHoldSettings();
  settings.timeConstantSec = candidate.timeConstantSec;
  settings.maximumPositivePitchRateRadPerSec =
      math::DegToRad(candidate.maximumPositiveRateDegPerSec);
  settings.maximumNegativePitchRateRadPerSec =
      math::DegToRad(candidate.maximumNegativeRateDegPerSec);
  settings.rateProportionalGain = candidate.p;
  settings.rateIntegralGain = candidate.i;
  settings.rateDerivativeGain = candidate.d;
  settings.rateFeedForwardGain = candidate.ff;
  settings.integratorLimit = candidate.integratorLimit;
  autopilot.SetPitchHoldSettings(settings);
  auto rollSettings = autopilot.GetRollHoldSettings();
  rollSettings.timeConstantSec = rollCandidate.timeConstantSec;
  rollSettings.rateProportionalGain = rollCandidate.p;
  rollSettings.rateIntegralGain = rollCandidate.i;
  rollSettings.rateDerivativeGain = rollCandidate.d;
  rollSettings.rateFeedForwardGain = rollCandidate.ff;
  autopilot.SetRollHoldSettings(rollSettings);
  const double targetRollRad =
      simulation.GetAircraft().GetProperties().Roll().Rad();
  autopilot.SetTargetRollRad(targetRollRad);
  autopilot.SetRollHoldEnabled(rollHoldEnabled);
  if (yawRateAugmented) {
    auto yawSettings = autopilot.GetYawRateSettings();
    yawSettings.rateProportionalGain = 0.8;
    yawSettings.rateIntegralGain = 0.0;
    yawSettings.rateDerivativeGain = 0.0;
    yawSettings.rateFeedForwardGain = 0.0;
    yawSettings.sideslipToYawRateGain = 8.0;
    autopilot.SetYawRateSettings(yawSettings);
  }
  autopilot.SetYawRateControlEnabled(yawRateControlEnabled);
  autopilot.SetTargetPitchRad(math::DegToRad(definition.sourcePitchDeg));
  autopilot.SetPitchHoldEnabled(true);
  manager.SetMode(control::FlightControlMode::Autopilot);

  std::vector<TransitionSample> preconditionSamples;
  preconditionSamples.reserve(
      static_cast<std::size_t>(TransitionPreconditionSec * hz));
  for (int tick = 0; tick < std::lround(TransitionPreconditionSec * hz);
      ++tick) {
    if (!simulation.Tick()) {
      throw std::runtime_error("Transition precondition tick failed");
    }
    preconditionSamples.push_back(CaptureTransitionSample(simulation,
        autopilot,
        static_cast<double>(tick) / hz));
  }

  autopilot.SetTargetPitchRad(math::DegToRad(definition.targetPitchDeg));
  passthrough.throttle =
      std::clamp(trim->throttle + throttlePitchGain * definition.targetPitchDeg,
          0.0,
          1.0);
  manager.GetManualController().SetCommandedInput(passthrough);
  std::vector<TransitionSample> responseSamples;
  responseSamples.reserve(
      static_cast<std::size_t>(TransitionObservationSec * hz));
  for (int tick = 0; tick < std::lround(TransitionObservationSec * hz);
      ++tick) {
    if (!simulation.Tick()) {
      throw std::runtime_error("Transition response tick failed");
    }
    responseSamples.push_back(CaptureTransitionSample(simulation,
        autopilot,
        static_cast<double>(tick + 1) / hz));
  }

  const TransitionMetrics metrics = EvaluateTransition(preconditionSamples,
      responseSamples,
      definition,
      math::RadToDeg(targetRollRad),
      hz);
  return {definition,
      candidate,
      rollCandidate,
      hz,
      airspeedKts,
      rollHoldEnabled,
      yawRateControlEnabled,
      metrics,
      std::move(preconditionSamples),
      std::move(responseSamples)};
}

std::vector<TransitionRun> RunAcceptance(Candidate candidate, double hz,
    double airspeedKts, bool rollHoldEnabled, bool yawRateControlEnabled,
    bool yawRateAugmented, const RollCandidate &rollCandidate,
    double throttlePitchGain) {
  constexpr std::array Transitions{
      TransitionDefinition{"0_to_pos5", 0.0, 5.0},
      TransitionDefinition{"0_to_neg5", 0.0, -5.0},
      TransitionDefinition{"pos5_to_0", 5.0, 0.0},
      TransitionDefinition{"neg5_to_0", -5.0, 0.0},
  };
  std::vector<TransitionRun> runs;
  runs.reserve(Transitions.size());
  for (const TransitionDefinition &definition : Transitions) {
    runs.push_back(ExecuteTransition(definition,
        candidate,
        hz,
        airspeedKts,
        rollHoldEnabled,
        yawRateControlEnabled,
        yawRateAugmented,
        rollCandidate,
        throttlePitchGain));
  }
  return runs;
}

void WriteAcceptance(const std::filesystem::path &path,
    std::span<const TransitionRun> runs) {
  std::ofstream output(path);
  output << "test,hz,initial_airspeed_kts,roll_hold_enabled,"
            "yaw_rate_control_enabled,source_pitch_deg,target_pitch_deg,tc,"
            "rmax_pos,rmax_neg,p,i,d,"
            "ff,imax,roll_tc,roll_p,roll_i,roll_d,roll_ff,overshoot_deg,"
            "settling_sec,steady_max_error_deg,"
            "steady_rms_error_deg,steady_max_q_deg_s,steady_rms_q_deg_s,"
            "steady_max_rate_error_deg_s,steady_max_roll_error_deg,"
            "max_roll_error_deg,max_roll_rate_deg_s,max_yaw_rate_deg_s,"
            "max_sideslip_deg,max_aileron_magnitude,max_roll_integrator,"
            "roll_saturation_duration_sec,roll_saturation_count,"
            "roll_integrator_limited_duration_sec,"
            "roll_integrator_limited_count,"
            "elevator_min,elevator_max,"
            "steady_elevator_range,saturation_duration_sec,saturation_count,"
            "steady_saturation_duration_sec,integrator_limited_duration_sec,"
            "integrator_limited_count,steady_integrator_limited_duration_sec,"
            "max_integrator,final_integrator,min_airspeed_kts,"
            "max_airspeed_kts,source_valid,required_pass,quality_pass\n";
  for (const TransitionRun &run : runs) {
    const Candidate &c = run.candidate;
    const RollCandidate &r = run.rollCandidate;
    const TransitionMetrics &m = run.metrics;
    output << run.definition.name << ',' << run.hz << ','
           << run.initialAirspeedKts << ',' << (run.rollHoldEnabled ? 1 : 0)
           << ',' << (run.yawRateControlEnabled ? 1 : 0) << ','
           << run.definition.sourcePitchDeg << ','
           << run.definition.targetPitchDeg << ',' << c.timeConstantSec << ','
           << c.maximumPositiveRateDegPerSec << ','
           << c.maximumNegativeRateDegPerSec << ',' << c.p << ',' << c.i << ','
           << c.d << ',' << c.ff << ',' << c.integratorLimit << ','
           << r.timeConstantSec << ',' << r.p << ',' << r.i << ',' << r.d << ','
           << r.ff << ',' << m.overshootDeg << ','
           << FiniteOr(m.settlingSec, -1.0) << ',' << m.steadyMaxPitchErrorDeg
           << ',' << m.steadyRmsPitchErrorDeg << ','
           << m.steadyMaxPitchRateDegPerSec << ','
           << m.steadyRmsPitchRateDegPerSec << ','
           << m.steadyMaxPitchRateErrorDegPerSec << ','
           << m.steadyMaxRollErrorDeg << ',' << m.maximumRollErrorDeg << ','
           << m.maximumRollRateDegPerSec << ',' << m.maximumYawRateDegPerSec
           << ',' << m.maximumSideslipDeg << ',' << m.maximumAileronMagnitude
           << ',' << m.maximumRollIntegratorMagnitude << ','
           << m.rollSaturationDurationSec << ',' << m.rollSaturationCount << ','
           << m.rollIntegratorLimitedDurationSec << ','
           << m.rollIntegratorLimitedCount << ',' << m.minimumElevator << ','
           << m.maximumElevator << ',' << m.steadyElevatorRange << ','
           << m.saturationDurationSec << ',' << m.saturationCount << ','
           << m.steadySaturationDurationSec << ','
           << m.integratorLimitedDurationSec << ',' << m.integratorLimitedCount
           << ',' << m.steadyIntegratorLimitedDurationSec << ','
           << m.maximumIntegratorMagnitude << ',' << m.finalIntegrator << ','
           << m.minimumAirspeedKts << ',' << m.maximumAirspeedKts << ','
           << (m.sourceConditionValid ? 1 : 0) << ','
           << (m.requiredPass ? 1 : 0) << ',' << (m.qualityPass ? 1 : 0)
           << '\n';
  }
}

void WriteAcceptanceTrace(const std::filesystem::path &path,
    std::span<const TransitionRun> runs) {
  std::ofstream output(path);
  output
      << "test,phase,time_sec,target_pitch_deg,actual_pitch_deg,actual_roll_"
         "deg,"
         "actual_roll_rate_deg_s,actual_yaw_rate_deg_s,sideslip_deg,"
         "pitch_error_deg,commanded_pitch_rate_deg_s,actual_pitch_rate_deg_s,"
         "pitch_rate_error_deg_s,elevator,aileron,rudder,p_term,i_term,d_term,"
         "ff_term,commanded_roll_rate_deg_s,roll_rate_error_deg_s,roll_p_term,"
         "roll_i_term,roll_d_term,roll_ff_term,roll_integrator,roll_saturated,"
         "roll_integrator_limited,"
         "integrator,saturated,integrator_limited,airspeed_kts\n";
  const auto writeSamples = [&](const TransitionRun &run,
                                std::string_view phase,
                                double targetPitchDeg,
                                std::span<const TransitionSample>
                                    samples) {
    for (const TransitionSample &sample : samples) {
      output << run.definition.name << ',' << phase << ',' << sample.timeSec
             << ',' << targetPitchDeg << ',' << sample.pitchDeg << ','
             << sample.rollDeg << ',' << sample.rollRateDegPerSec << ','
             << sample.yawRateDegPerSec << ',' << sample.sideslipDeg << ','
             << sample.pitchErrorDeg << ',' << sample.pitchRateSetpointDegPerSec
             << ',' << sample.pitchRateDegPerSec << ','
             << sample.pitchRateErrorDegPerSec << ',' << sample.elevator << ','
             << sample.aileron << ',' << sample.rudder << ','
             << sample.proportionalTerm << ',' << sample.integralTerm << ','
             << sample.derivativeTerm << ',' << sample.feedForwardTerm << ','
             << sample.rollRateSetpointDegPerSec << ','
             << sample.rollRateErrorDegPerSec << ','
             << sample.rollProportionalTerm << ',' << sample.rollIntegralTerm
             << ',' << sample.rollDerivativeTerm << ','
             << sample.rollFeedForwardTerm << ',' << sample.rollIntegrator
             << ',' << (sample.rollSaturated ? 1 : 0) << ','
             << (sample.rollIntegratorLimited ? 1 : 0) << ','
             << sample.integrator << ',' << (sample.saturated ? 1 : 0) << ','
             << (sample.integratorLimited ? 1 : 0) << ',' << sample.airspeedKts
             << '\n';
    }
  };
  for (const TransitionRun &run : runs) {
    writeSamples(run,
        "precondition",
        run.definition.sourcePitchDeg,
        run.preconditionSamples);
    writeSamples(run,
        "response",
        run.definition.targetPitchDeg,
        run.responseSamples);
  }
}

void PrintAcceptance(std::span<const TransitionRun> runs) {
  std::cout << std::fixed << std::setprecision(4);
  for (const TransitionRun &run : runs) {
    const TransitionMetrics &m = run.metrics;
    std::cout << run.definition.name << ": overshoot=" << m.overshootDeg
              << " settling=" << FiniteOr(m.settlingSec, -1.0)
              << " steady_max_error=" << m.steadyMaxPitchErrorDeg
              << " steady_rms_error=" << m.steadyRmsPitchErrorDeg
              << " steady_max_q=" << m.steadyMaxPitchRateDegPerSec
              << " steady_rms_q=" << m.steadyRmsPitchRateDegPerSec
              << " max_rate_error=" << m.steadyMaxPitchRateErrorDegPerSec
              << " max_roll_error=" << m.steadyMaxRollErrorDeg
              << " roll_saturation=" << m.rollSaturationDurationSec << "s/"
              << m.rollSaturationCount
              << " roll_integrator_limit=" << m.rollIntegratorLimitedDurationSec
              << "s/" << m.rollIntegratorLimitedCount
              << " saturation=" << m.saturationDurationSec << "s/"
              << m.saturationCount
              << " integrator_limit=" << m.integratorLimitedDurationSec << "s/"
              << m.integratorLimitedCount << " airspeed=["
              << m.minimumAirspeedKts << ',' << m.maximumAirspeedKts << "]"
              << " required=" << (m.requiredPass ? "PASS" : "FAIL")
              << " quality=" << (m.qualityPass ? "PASS" : "MISS") << '\n';
  }
}
} // namespace

int main(int argc, char **argv) {
  std::filesystem::path output = "build/test-results/px4-pitch-tuning";
  Candidate acceptanceCandidate{0.2, 4.5, 4.5, 0.0, 1.2, 14.0, 10.0, 0.4};
  bool acceptanceOnly = false;
  double acceptanceHz = 120.0;
  double acceptanceAirspeedKts = 80.0;
  bool acceptanceRollHoldEnabled = true;
  bool acceptanceYawRateControlEnabled = false;
  bool acceptanceYawRateAugmented = false;
  RollCandidate acceptanceRollCandidate;
  double acceptanceThrottlePitchGain = 0.0;

  try {
    for (int index = 1; index < argc; ++index) {
      const std::string argument = argv[index];
      if (argument == "--help") {
        std::cout << "Usage: px4_pitch_tuning_probe [--output DIRECTORY] "
                     "[--acceptance-only [--tc VALUE] [--rmax-pos VALUE] "
                     "[--rmax-neg VALUE] [--p VALUE] [--i VALUE] [--d VALUE] "
                     "[--ff VALUE] [--imax VALUE] [--hz VALUE] "
                     "[--airspeed VALUE] [--roll-hold-off] "
                     "[--yaw-rate-on] [--yaw-augmented] [--roll-tc VALUE] "
                     "[--roll-p VALUE] [--roll-i VALUE] [--roll-d VALUE] "
                     "[--roll-ff VALUE] "
                     "[--throttle-pitch-gain VALUE]]\n";
        return EXIT_SUCCESS;
      }
      if (argument == "--acceptance-only") {
        acceptanceOnly = true;
        continue;
      }
      if (argument == "--roll-hold-off") {
        acceptanceRollHoldEnabled = false;
        continue;
      }
      if (argument == "--yaw-rate-on") {
        acceptanceYawRateControlEnabled = true;
        continue;
      }
      if (argument == "--yaw-augmented") {
        acceptanceYawRateControlEnabled = true;
        acceptanceYawRateAugmented = true;
        continue;
      }
      if (argument == "--output" && index + 1 < argc) {
        output = argv[++index];
        continue;
      }

      const auto readDouble = [&](double &destination) {
        if (index + 1 >= argc) {
          throw std::invalid_argument("Missing value for " + argument);
        }
        std::size_t parsedCharacters = 0;
        const std::string value = argv[++index];
        destination = std::stod(value, &parsedCharacters);
        if (parsedCharacters != value.size() || !std::isfinite(destination)) {
          throw std::invalid_argument(
              "Invalid value for " + argument + ": " + value);
        }
      };
      if (argument == "--tc") {
        readDouble(acceptanceCandidate.timeConstantSec);
      } else if (argument == "--rmax-pos") {
        readDouble(acceptanceCandidate.maximumPositiveRateDegPerSec);
      } else if (argument == "--rmax-neg") {
        readDouble(acceptanceCandidate.maximumNegativeRateDegPerSec);
      } else if (argument == "--p") {
        readDouble(acceptanceCandidate.p);
      } else if (argument == "--i") {
        readDouble(acceptanceCandidate.i);
      } else if (argument == "--d") {
        readDouble(acceptanceCandidate.d);
      } else if (argument == "--ff") {
        readDouble(acceptanceCandidate.ff);
      } else if (argument == "--imax") {
        readDouble(acceptanceCandidate.integratorLimit);
      } else if (argument == "--hz") {
        readDouble(acceptanceHz);
      } else if (argument == "--airspeed") {
        readDouble(acceptanceAirspeedKts);
      } else if (argument == "--roll-tc") {
        readDouble(acceptanceRollCandidate.timeConstantSec);
      } else if (argument == "--roll-p") {
        readDouble(acceptanceRollCandidate.p);
      } else if (argument == "--roll-i") {
        readDouble(acceptanceRollCandidate.i);
      } else if (argument == "--roll-d") {
        readDouble(acceptanceRollCandidate.d);
      } else if (argument == "--roll-ff") {
        readDouble(acceptanceRollCandidate.ff);
      } else if (argument == "--throttle-pitch-gain") {
        readDouble(acceptanceThrottlePitchGain);
      } else {
        throw std::invalid_argument("Unknown argument: " + argument);
      }
    }

    std::filesystem::create_directories(output);
    if (acceptanceOnly) {
      if (acceptanceHz <= 0.0 || acceptanceAirspeedKts <= 0.0) {
        throw std::invalid_argument("Hz and airspeed must be positive");
      }
      const std::vector<TransitionRun> runs = RunAcceptance(acceptanceCandidate,
          acceptanceHz,
          acceptanceAirspeedKts,
          acceptanceRollHoldEnabled,
          acceptanceYawRateControlEnabled,
          acceptanceYawRateAugmented,
          acceptanceRollCandidate,
          acceptanceThrottlePitchGain);
      WriteAcceptance(output / "acceptance.csv", runs);
      WriteAcceptanceTrace(output / "acceptance_trace.csv", runs);
      PrintAcceptance(runs);
      return EXIT_SUCCESS;
    }

    const std::vector<RankedCandidate> ranked = SweepNominal();
    const std::vector<Run> validation = ValidateCandidates(ranked);
    WriteRuns(output / "robust_validation.csv", validation);
    PrintSummary(ranked, validation);
  } catch (const std::exception &error) {
    std::cerr << "px4_pitch_tuning_probe: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
