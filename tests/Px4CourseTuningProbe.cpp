#include "common/math/Math.hpp"
#include "sim/FDMState.hpp"
#include "sim/InitialCondition.hpp"
#include "sim/Simulation.hpp"
#include "sim/control/FlightControlManager.hpp"
#include "sim/control/FlightControlMode.hpp"
#include "sim/gnc/autopilot/PX4Autopilot.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
constexpr double StepTimeSec = 3.0;
constexpr double RunDurationSec = 18.0;

struct Candidate {
  double periodSec{};
  double damping{};
  double maxRollDeg{};
  double slewDegPerSec{};
};

struct ProbeOptions {
  std::filesystem::path output = "build/test-results/px4-course-tuning";
  std::optional<Candidate> candidate;
  bool validationOnly = false;
};

struct Sample {
  double timeSec{};
  double targetCourseDeg{};
  double courseDeg{};
  double courseErrorDeg{};
  double rawRollSetpointDeg{};
  double rollSetpointDeg{};
  double rollDeg{};
  double rollRateDegPerSec{};
  double yawRateDegPerSec{};
  double betaDeg{};
  double aileron{};
  double rudder{};
  bool rollLimited{};
  bool slewLimited{};
  bool saturated{};
};

struct Metrics {
  double peakCourseErrorDeg{};
  double settlingSec{std::numeric_limits<double>::infinity()};
  double overshootDeg{};
  double steadyCourseErrorDeg{};
  double rmsCourseErrorDeg{};
  double maxCommandedRollDeg{};
  double maxActualRollDeg{};
  double peakCommandedRollRateDegPerSec{};
  double peakActualRollRateDegPerSec{};
  double rmsRollTrackingErrorDeg{};
  double peakYawRateDegPerSec{};
  double rmsYawRateDegPerSec{};
  double peakBetaDeg{};
  double rmsBetaDeg{};
  double maxAileron{};
  double maxRudder{};
  double actuatorVariationPerSec{};
  double rollLimitedFraction{};
  double slewLimitedFraction{};
  double saturationFraction{};
  double averageTickUsec{};
  double p95TickUsec{};
  double p99TickUsec{};
  bool saturated{};
  double score{};
};

struct Run {
  std::string scenario;
  double hz{};
  double airspeedKts{};
  double stepDeg{};
  Candidate candidate;
  Metrics metrics;
  std::vector<Sample> samples;
};

struct RollRegressionMetrics {
  std::string scenario;
  double hz{};
  double commandedRollDeg{};
  double overshootDeg{};
  double settlingSec{std::numeric_limits<double>::infinity()};
  double steadyErrorDeg{};
  double peakRollRateDegPerSec{};
  double minimumRollRateDegPerSec{};
  double rmsRollRateErrorDegPerSec{};
  double peakYawRateDegPerSec{};
  double rmsYawRateDegPerSec{};
  double peakBetaDeg{};
  double rmsBetaDeg{};
  double maxAileron{};
  double maxRudder{};
  bool saturated{};
};

double Square(double value) { return value * value; }

double Rms(const std::vector<double> &values) {
  if (values.empty()) {
    return 0.0;
  }
  return std::sqrt(
      std::accumulate(values.begin(),
          values.end(),
          0.0,
          [](double sum, double value) { return sum + Square(value); })
      / static_cast<double>(values.size()));
}

double Mean(const std::vector<double> &values) {
  return values.empty() ? 0.0
                        : std::accumulate(values.begin(), values.end(), 0.0)
                              / static_cast<double>(values.size());
}

std::string Format(double value, int precision = 3) {
  if (!std::isfinite(value)) {
    return "not-settled";
  }
  std::ostringstream output;
  output << std::fixed << std::setprecision(precision) << value;
  return output.str();
}

control::FlightControlManager &Manager(sim::Simulation &simulation) {
  return simulation.GetFlightControlManager();
}

gnc::PX4Autopilot &Autopilot(sim::Simulation &simulation) {
  return dynamic_cast<gnc::PX4Autopilot &>(Manager(simulation).GetAutopilot());
}

Metrics Evaluate(const std::vector<Sample> &samples,
    const std::vector<double> &tickUsec, double initialCourseDeg,
    double stepDeg) {
  Metrics metrics;
  std::vector<double> errors;
  std::vector<double> tailErrors;
  std::vector<double> rollErrors;
  std::vector<double> yawRates;
  std::vector<double> betas;
  double lastOutsideTimeSec = StepTimeSec;
  bool outsideAtEnd = false;
  double previousAileron = samples.front().aileron;
  double previousRudder = samples.front().rudder;
  double previousRollSetpointDeg = samples.front().rollSetpointDeg;
  double previousTimeSec = samples.front().timeSec;
  std::size_t evaluatedSamples = 0;
  std::size_t rollLimitedSamples = 0;
  std::size_t slewLimitedSamples = 0;
  std::size_t saturatedSamples = 0;
  for (const Sample &sample : samples) {
    if (sample.timeSec < StepTimeSec) {
      previousAileron = sample.aileron;
      previousRudder = sample.rudder;
      previousRollSetpointDeg = sample.rollSetpointDeg;
      previousTimeSec = sample.timeSec;
      continue;
    }
    ++evaluatedSamples;
    metrics.peakCourseErrorDeg =
        std::max(metrics.peakCourseErrorDeg, std::abs(sample.courseErrorDeg));
    errors.push_back(sample.courseErrorDeg);
    rollErrors.push_back(sample.rollSetpointDeg - sample.rollDeg);
    yawRates.push_back(sample.yawRateDegPerSec);
    betas.push_back(sample.betaDeg);
    metrics.maxCommandedRollDeg =
        std::max(metrics.maxCommandedRollDeg, std::abs(sample.rollSetpointDeg));
    metrics.maxActualRollDeg =
        std::max(metrics.maxActualRollDeg, std::abs(sample.rollDeg));
    const double sampleDtSec = sample.timeSec - previousTimeSec;
    if (sampleDtSec > 0.0) {
      metrics.peakCommandedRollRateDegPerSec =
          std::max(metrics.peakCommandedRollRateDegPerSec,
              std::abs(sample.rollSetpointDeg - previousRollSetpointDeg)
                  / sampleDtSec);
    }
    metrics.peakActualRollRateDegPerSec =
        std::max(metrics.peakActualRollRateDegPerSec,
            std::abs(sample.rollRateDegPerSec));
    metrics.peakYawRateDegPerSec = std::max(metrics.peakYawRateDegPerSec,
        std::abs(sample.yawRateDegPerSec));
    metrics.peakBetaDeg =
        std::max(metrics.peakBetaDeg, std::abs(sample.betaDeg));
    metrics.maxAileron = std::max(metrics.maxAileron, std::abs(sample.aileron));
    metrics.maxRudder = std::max(metrics.maxRudder, std::abs(sample.rudder));
    metrics.saturated = metrics.saturated || sample.saturated;
    rollLimitedSamples += sample.rollLimited ? 1U : 0U;
    slewLimitedSamples += sample.slewLimited ? 1U : 0U;
    saturatedSamples += sample.saturated ? 1U : 0U;
    metrics.actuatorVariationPerSec +=
        std::abs(sample.aileron - previousAileron)
        + std::abs(sample.rudder - previousRudder);
    previousAileron = sample.aileron;
    previousRudder = sample.rudder;
    previousRollSetpointDeg = sample.rollSetpointDeg;
    previousTimeSec = sample.timeSec;
    const double traveledDeg =
        math::RadToDeg(math::DeltaAngleRad(math::DegToRad(initialCourseDeg),
            math::DegToRad(sample.courseDeg)));
    const double signedOvershoot =
        std::copysign(1.0, stepDeg) * (traveledDeg - stepDeg);
    metrics.overshootDeg = std::max(metrics.overshootDeg, signedOvershoot);
    outsideAtEnd = std::abs(sample.courseErrorDeg) > 1.0;
    if (outsideAtEnd) {
      lastOutsideTimeSec = sample.timeSec;
    }
    if (sample.timeSec >= RunDurationSec - 3.0) {
      tailErrors.push_back(sample.courseErrorDeg);
    }
  }
  metrics.rmsCourseErrorDeg = Rms(errors);
  metrics.steadyCourseErrorDeg = Mean(tailErrors);
  metrics.rmsRollTrackingErrorDeg = Rms(rollErrors);
  metrics.rmsYawRateDegPerSec = Rms(yawRates);
  metrics.rmsBetaDeg = Rms(betas);
  metrics.actuatorVariationPerSec /= RunDurationSec - StepTimeSec;
  if (evaluatedSamples > 0) {
    metrics.rollLimitedFraction =
        static_cast<double>(rollLimitedSamples) / evaluatedSamples;
    metrics.slewLimitedFraction =
        static_cast<double>(slewLimitedSamples) / evaluatedSamples;
    metrics.saturationFraction =
        static_cast<double>(saturatedSamples) / evaluatedSamples;
  }
  if (!outsideAtEnd) {
    metrics.settlingSec = std::max(0.0, lastOutsideTimeSec - StepTimeSec);
  }
  metrics.averageTickUsec = Mean(tickUsec);
  std::vector<double> sortedTickUsec = tickUsec;
  std::sort(sortedTickUsec.begin(), sortedTickUsec.end());
  const auto percentile = [&sortedTickUsec](double fraction) {
    const std::size_t index = static_cast<std::size_t>(
        std::floor(fraction * static_cast<double>(sortedTickUsec.size() - 1)));
    return sortedTickUsec[index];
  };
  metrics.p95TickUsec = percentile(0.95);
  metrics.p99TickUsec = percentile(0.99);
  const auto cost = [](double value, double scale) {
    return std::min(std::abs(value) / scale, 5.0);
  };
  const std::array costs{
      cost(std::isfinite(metrics.settlingSec) ? metrics.settlingSec : 15.0,
          8.0),
      cost(metrics.overshootDeg, 2.0),
      cost(metrics.steadyCourseErrorDeg, 1.0),
      cost(metrics.rmsCourseErrorDeg, std::max(2.0, std::abs(stepDeg) * 0.5)),
      cost(metrics.rmsRollTrackingErrorDeg, 2.0),
      cost(metrics.maxActualRollDeg, 25.0),
      cost(metrics.rmsYawRateDegPerSec, 2.0),
      cost(metrics.rmsBetaDeg, 1.0),
      cost(metrics.actuatorVariationPerSec, 0.5),
  };
  metrics.score = std::accumulate(costs.begin(), costs.end(), 0.0)
                      / static_cast<double>(costs.size())
                  + (metrics.saturated ? 2.0 : 0.0);
  return metrics;
}

Run Execute(std::string scenario, double hz, double airspeedKts,
    double initialHeadingDeg, double targetCourseDeg, Candidate candidate,
    bool retainSamples, bool enableAtStep = false) {
  sim::Simulation simulation(std::make_unique<gnc::PX4Autopilot>());
  if (!simulation.Initialize(opts::simulation::AircraftName, hz)) {
    throw std::runtime_error("Failed to initialize course tuning simulation");
  }
  sim::InitialCondition initial = simulation.GetDefaultInitialCondition();
  initial.calibratedAirspeedMps =
      math::KnotsToMetersPerSecond(airspeedKts);
  initial.headingRad = math::DegToRad(initialHeadingDeg);
  if (!simulation.Reset(initial)) {
    throw std::runtime_error("Failed to reset course tuning initial state");
  }

  auto &autopilot = Autopilot(simulation);
  auto courseSettings = autopilot.GetCourseHoldSettings();
  courseSettings.guidancePeriodSec = candidate.periodSec;
  courseSettings.guidanceDampingRatio = candidate.damping;
  courseSettings.maxRollRad = math::DegToRad(candidate.maxRollDeg);
  courseSettings.maxRollSetpointRateRadPerSec =
      math::DegToRad(candidate.slewDegPerSec);
  autopilot.SetCourseHoldSettings(courseSettings);

  auto yawSettings = autopilot.GetYawRateSettings();
  yawSettings.rateProportionalGain = 0.8;
  yawSettings.rateIntegralGain = 0.0;
  yawSettings.rateDerivativeGain = 0.0;
  yawSettings.rateFeedForwardGain = 0.0;
  yawSettings.sideslipToYawRateGain = 8.0;
  autopilot.SetYawRateSettings(yawSettings);

  const double initialCourseDeg =
      simulation.GetAircraft().GetProperties().Course().Deg();
  autopilot.SetTargetCourseRad(
      math::DegToRad(enableAtStep ? targetCourseDeg : initialCourseDeg));
  autopilot.SetTargetRollRad(
      simulation.GetAircraft().GetProperties().Roll().Rad());
  autopilot.SetRollHoldEnabled(true);
  autopilot.SetCourseHoldEnabled(!enableAtStep);
  autopilot.SetYawRateControlEnabled(true);
  Manager(simulation).SetMode(control::FlightControlMode::Autopilot);

  std::vector<Sample> samples;
  std::vector<double> tickUsec;
  samples.reserve(static_cast<std::size_t>(RunDurationSec * hz));
  tickUsec.reserve(samples.capacity());
  for (int tick = 0; tick < std::lround(RunDurationSec * hz); ++tick) {
    const double timeSec = static_cast<double>(tick) / hz;
    if (timeSec >= StepTimeSec) {
      if (enableAtStep) {
        autopilot.SetCourseHoldEnabled(true);
        enableAtStep = false;
      } else {
        autopilot.SetTargetCourseRad(math::DegToRad(targetCourseDeg));
      }
    }
    const auto tickStart = std::chrono::steady_clock::now();
    if (!simulation.Tick()) {
      throw std::runtime_error("Course tuning simulation tick failed");
    }
    const auto tickEnd = std::chrono::steady_clock::now();
    tickUsec.push_back(
        std::chrono::duration<double, std::micro>(tickEnd - tickStart).count());
    const auto &course = autopilot.GetCourseHoldDiagnostics();
    const auto &roll = autopilot.GetRollHoldDiagnostics();
    const auto &yaw = autopilot.GetYawRateDiagnostics();
    const sim::FDMState state = simulation.GetAircraft().ExtractFDMState(
        sim::FDMStateFlags::State | sim::FDMStateFlags::Controls);
    samples.push_back({simulation.GetTime(),
        math::RadToDeg(course.targetCourseRad),
        math::RadToDeg(course.currentCourseRad),
        math::RadToDeg(course.courseErrorRad),
        math::RadToDeg(course.rawRollSetpointRad),
        math::RadToDeg(course.limitedRollSetpointRad),
        math::RadToDeg(state.state.attitudeRad[0]),
        math::RadToDeg(state.state.bodyAngularRatesRadPerSec[0]),
        math::RadToDeg(state.state.bodyAngularRatesRadPerSec[2]),
        simulation.GetAircraft().GetProperties().Beta().Deg(),
        roll.aileronCommand,
        yaw.rudderCommand,
        course.rollLimited,
        course.rollSetpointRateLimited,
        roll.positiveSaturation || roll.negativeSaturation
            || yaw.positiveSaturation || yaw.negativeSaturation});
  }

  const double stepDeg =
      math::RadToDeg(math::DeltaAngleRad(math::DegToRad(initialCourseDeg),
          math::DegToRad(targetCourseDeg)));
  Run run{std::move(scenario), hz, airspeedKts, stepDeg, candidate};
  run.metrics = Evaluate(samples, tickUsec, initialCourseDeg, stepDeg);
  if (retainSamples) {
    run.samples = std::move(samples);
  }
  return run;
}

RollRegressionMetrics ExecuteRollRegression(std::string scenario, double hz,
    double rollStepDeg, bool directRatePulse) {
  sim::Simulation simulation(std::make_unique<gnc::PX4Autopilot>());
  if (!simulation.Initialize(opts::simulation::AircraftName, hz)) {
    throw std::runtime_error("Failed to initialize roll regression simulation");
  }
  auto &autopilot = Autopilot(simulation);
  auto rollSettings = autopilot.GetRollHoldSettings();
  rollSettings.rateFeedForwardGain = 1.20;
  rollSettings.rateProportionalGain = 1.90;
  rollSettings.rateIntegralGain = 0.25;
  rollSettings.rateDerivativeGain = 0.0;
  rollSettings.directRollRateTestEnabled = directRatePulse;
  autopilot.SetRollHoldSettings(rollSettings);
  auto yawSettings = autopilot.GetYawRateSettings();
  yawSettings.rateProportionalGain = 0.8;
  yawSettings.rateIntegralGain = 0.0;
  yawSettings.rateDerivativeGain = 0.0;
  yawSettings.rateFeedForwardGain = 0.0;
  yawSettings.sideslipToYawRateGain = 8.0;
  autopilot.SetYawRateSettings(yawSettings);
  const double initialRollRad =
      simulation.GetAircraft().GetProperties().Roll().Rad();
  autopilot.SetTargetRollRad(initialRollRad);
  autopilot.SetRollHoldEnabled(true);
  autopilot.SetYawRateControlEnabled(true);
  Manager(simulation).SetMode(control::FlightControlMode::Autopilot);

  std::vector<double> tailErrors;
  std::vector<double> rateErrors;
  std::vector<double> yawRates;
  std::vector<double> betas;
  RollRegressionMetrics result;
  result.scenario = std::move(scenario);
  result.hz = hz;
  result.commandedRollDeg = rollStepDeg;
  result.minimumRollRateDegPerSec = std::numeric_limits<double>::infinity();
  double lastOutsideTimeSec = StepTimeSec;
  bool outsideAtEnd = false;
  for (int tick = 0; tick < std::lround(RunDurationSec * hz); ++tick) {
    const double timeSec = static_cast<double>(tick) / hz;
    if (directRatePulse) {
      rollSettings.directRollRateCommandRadPerSec =
          timeSec >= StepTimeSec && timeSec < StepTimeSec + 2.0
              ? math::DegToRad(5.0)
              : 0.0;
      autopilot.SetRollHoldSettings(rollSettings);
    } else if (timeSec >= StepTimeSec) {
      autopilot.SetTargetRollRad(initialRollRad + math::DegToRad(rollStepDeg));
    }
    if (!simulation.Tick()) {
      throw std::runtime_error("Roll regression tick failed");
    }
    if (timeSec < StepTimeSec) {
      continue;
    }
    const auto &roll = autopilot.GetRollHoldDiagnostics();
    const auto &yaw = autopilot.GetYawRateDiagnostics();
    const double actualRollDeg =
        simulation.GetAircraft().GetProperties().Roll().Deg();
    const double relativeRollDeg =
        actualRollDeg - math::RadToDeg(initialRollRad);
    const double pDegPerSec =
        simulation.GetAircraft().GetProperties().P().DegPerSec();
    const double rDegPerSec =
        simulation.GetAircraft().GetProperties().R().DegPerSec();
    const double betaDeg =
        simulation.GetAircraft().GetProperties().Beta().Deg();
    const double targetDeg = directRatePulse ? 0.0 : rollStepDeg;
    const double rollErrorDeg = targetDeg - relativeRollDeg;
    if (!directRatePulse) {
      result.overshootDeg = std::max(result.overshootDeg,
          std::copysign(1.0, rollStepDeg) * (relativeRollDeg - rollStepDeg));
      outsideAtEnd = std::abs(rollErrorDeg) > 0.5;
      if (outsideAtEnd) {
        lastOutsideTimeSec = timeSec;
      }
      if (timeSec >= RunDurationSec - 3.0) {
        tailErrors.push_back(rollErrorDeg);
      }
    }
    rateErrors.push_back(math::RadToDeg(roll.bodyRateErrorRadPerSec));
    yawRates.push_back(rDegPerSec);
    betas.push_back(betaDeg);
    result.peakRollRateDegPerSec =
        std::max(result.peakRollRateDegPerSec, pDegPerSec);
    result.minimumRollRateDegPerSec =
        std::min(result.minimumRollRateDegPerSec, pDegPerSec);
    result.peakYawRateDegPerSec =
        std::max(result.peakYawRateDegPerSec, std::abs(rDegPerSec));
    result.peakBetaDeg = std::max(result.peakBetaDeg, std::abs(betaDeg));
    result.maxAileron =
        std::max(result.maxAileron, std::abs(roll.aileronCommand));
    result.maxRudder = std::max(result.maxRudder, std::abs(yaw.rudderCommand));
    result.saturated = result.saturated || roll.positiveSaturation
                       || roll.negativeSaturation || yaw.positiveSaturation
                       || yaw.negativeSaturation;
  }
  if (!directRatePulse && !outsideAtEnd) {
    result.settlingSec = std::max(0.0, lastOutsideTimeSec - StepTimeSec);
  }
  result.steadyErrorDeg = Mean(tailErrors);
  result.rmsRollRateErrorDegPerSec = Rms(rateErrors);
  result.rmsYawRateDegPerSec = Rms(yawRates);
  result.rmsBetaDeg = Rms(betas);
  return result;
}

void WriteSweep(const std::filesystem::path &path,
    const std::vector<Run> &runs) {
  std::ofstream out(path);
  out << "scenario,hz,airspeed_kts,step_deg,period_sec,damping,candidate_max_"
         "roll_deg,"
         "slew_deg_s,peak_error_deg,settling_sec,overshoot_deg,steady_error_"
         "deg,"
         "rms_error_deg,max_roll_sp_deg,max_actual_roll_deg,rms_roll_error_deg,"
         "peak_r_"
         "deg_s,peak_roll_sp_rate_deg_s,peak_p_deg_s,"
         "rms_r_deg_s,peak_beta_deg,rms_beta_deg,max_aileron,max_rudder,"
         "variation_s,roll_limited_fraction,slew_limited_fraction,"
         "saturation_fraction,saturated,score,avg_tick_us,p95_tick_us,p99_"
         "tick_us\n";
  for (const Run &run : runs) {
    const Metrics &m = run.metrics;
    out << run.scenario << ',' << run.hz << ',' << run.airspeedKts << ','
        << run.stepDeg << ',' << run.candidate.periodSec << ','
        << run.candidate.damping << ',' << run.candidate.maxRollDeg << ','
        << run.candidate.slewDegPerSec << ',' << m.peakCourseErrorDeg << ','
        << m.settlingSec << ',' << m.overshootDeg << ','
        << m.steadyCourseErrorDeg << ',' << m.rmsCourseErrorDeg << ','
        << m.maxCommandedRollDeg << ',' << m.maxActualRollDeg << ','
        << m.rmsRollTrackingErrorDeg << ',' << m.peakYawRateDegPerSec << ','
        << m.peakCommandedRollRateDegPerSec << ','
        << m.peakActualRollRateDegPerSec << ',' << m.rmsYawRateDegPerSec << ','
        << m.peakBetaDeg << ',' << m.rmsBetaDeg << ',' << m.maxAileron << ','
        << m.maxRudder << ',' << m.actuatorVariationPerSec << ','
        << m.rollLimitedFraction << ',' << m.slewLimitedFraction << ','
        << m.saturationFraction << ',' << (m.saturated ? 1 : 0) << ','
        << m.score << ',' << m.averageTickUsec << ',' << m.p95TickUsec << ','
        << m.p99TickUsec << '\n';
  }
}

void WriteSamples(const std::filesystem::path &path,
    const std::vector<Run> &runs) {
  std::ofstream out(path);
  out << "candidate,scenario,hz,time_sec,target_course_deg,course_deg,"
         "course_error_deg,raw_roll_sp_deg,limited_roll_sp_deg,roll_deg,p_deg_"
         "s,"
         "r_deg_s,beta_deg,aileron,rudder,roll_limited,slew_limited,"
         "saturated\n";
  for (std::size_t candidate = 0; candidate < runs.size(); ++candidate) {
    for (const Sample &sample : runs[candidate].samples) {
      out << candidate + 1 << ',' << runs[candidate].scenario << ','
          << runs[candidate].hz << ',' << sample.timeSec << ','
          << sample.targetCourseDeg << ',' << sample.courseDeg << ','
          << sample.courseErrorDeg << ',' << sample.rawRollSetpointDeg << ','
          << sample.rollSetpointDeg << ',' << sample.rollDeg << ','
          << sample.rollRateDegPerSec << ',' << sample.yawRateDegPerSec << ','
          << sample.betaDeg << ',' << sample.aileron << ',' << sample.rudder
          << ',' << sample.rollLimited << ',' << sample.slewLimited << ','
          << sample.saturated << '\n';
    }
  }
}

void WriteHeatmap(const std::filesystem::path &path,
    const std::vector<Run> &sweep) {
  constexpr std::array periods{8.0, 10.0, 14.0};
  constexpr std::array dampingValues{0.5, 0.7, 0.9};
  std::ofstream out(path);
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"520\" "
         "height=\"400\">"
         "<rect width=\"100%\" height=\"100%\" fill=\"#10151d\"/>"
         "<text x=\"20\" y=\"28\" fill=\"white\">Mean score by NPFG period / "
         "damping</text>";
  for (std::size_t row = 0; row < dampingValues.size(); ++row) {
    for (std::size_t column = 0; column < periods.size(); ++column) {
      std::vector<double> scores;
      for (const Run &run : sweep) {
        if (run.candidate.periodSec == periods[column]
            && run.candidate.damping == dampingValues[row]) {
          scores.push_back(run.metrics.score);
        }
      }
      const double score = Mean(scores);
      const int red = static_cast<int>(std::clamp(score / 2.0, 0.0, 1.0) * 210);
      const int green = 210 - red;
      const int x = 100 + static_cast<int>(column) * 120;
      const int y = 55 + static_cast<int>(row) * 95;
      out << "<rect x=\"" << x << "\" y=\"" << y
          << "\" width=\"105\" height=\"80\" fill=\"rgb(" << red << ',' << green
          << ",70)\"/><text x=\"" << x + 28 << "\" y=\"" << y + 45
          << "\" fill=\"white\">" << Format(score, 2) << "</text>";
    }
    out << "<text x=\"18\" y=\"" << 103 + static_cast<int>(row) * 95
        << "\" fill=\"white\">z=" << dampingValues[row] << "</text>";
  }
  for (std::size_t column = 0; column < periods.size(); ++column) {
    out << "<text x=\"" << 130 + static_cast<int>(column) * 120
        << "\" y=\"370\" fill=\"white\">T=" << periods[column] << "s</text>";
  }
  out << "</svg>\n";
}

void WriteReport(const std::filesystem::path &path, const Candidate &selected,
    const std::vector<Run> &robustness,
    const std::vector<RollRegressionMetrics> &rollRegression,
    bool deterministic) {
  std::ofstream out(path);
  out << "# PX4 Course Hold / 120 Hz Validation Report\n\n"
      << "## 1. Legacy removal and 120 Hz architecture\n\n"
      << "The UAVBook PI CourseHoldController is removed. One canonical "
         "The configured simulation timestep drives controller OnTick, JSBSim "
         "Setdt/Run, "
         "JSBSim actuator dynamics, and post-step telemetry. The production "
         "default is 120 Hz (8.333 ms); GUI rendering remains independent. "
         "Headless scenarios derive the same physics/controller rate from "
         "dt_sec. Previously, the configured rate reached JSBSim only after "
         "LoadModel; JSBSim FCS components cache channel dt while loading, so "
         "non-default actuator/filter timing could retain the library default. "
         "Setdt now runs before LoadModel, verified by the C172x 1.57 rad/s "
         "aileron slew test at both 30 and 120 Hz. Telemetry is generated once "
         "per post-physics tick using simulation time; Monitor/UI rendering "
         "continues on its independent GUI frame loop.\n\n"
      << "## 2. PX4 reference and signal flow\n\n"
      << "The implementation extracts the constant-direction subset of PX4 "
         "mainline DirectionalGuidance/AirspeedDirectionController at commit "
         "89f81c0f2ab354bf3890d29d3698c7e68f89ee6b. Direction error becomes "
         "lateral acceleration, then `phi_sp=atan(a_lat/g)` with bank and slew "
         "constraints. Course Hold outputs only phi_sp; the unchanged Roll "
         "Hold "
         "outputs aileron and the unchanged yaw augmentation outputs "
         "rudder.\n\n"
      << "Sources: "
         "[DirectionalGuidance.cpp](https://github.com/PX4/PX4-Autopilot/blob/"
         "89f81c0f2ab354bf3890d29d3698c7e68f89ee6b/src/lib/npfg/"
         "DirectionalGuidance.cpp), "
         "[AirspeedDirectionController.cpp](https://github.com/PX4/"
         "PX4-Autopilot/blob/89f81c0f2ab354bf3890d29d3698c7e68f89ee6b/src/lib/"
         "npfg/AirspeedDirectionController.cpp), "
         "and "
         "[FwLateralLongitudinalControl.cpp](https://github.com/PX4/"
         "PX4-Autopilot/blob/89f81c0f2ab354bf3890d29d3698c7e68f89ee6b/src/"
         "modules/fw_lateral_longitudinal_control/"
         "FwLateralLongitudinalControl.cpp).\n\n"
      << "## 3. Minimal controller equation\n\n"
      << "`k_dir=4*pi*zeta/T`, `v_cross=v_g x unit(chi_sp)`, "
         "`a_lat=clamp(k_dir*v_cross, +/-g*tan(phi_max))`, "
         "`phi_sp=slew(atan(a_lat/g))`. Error telemetry uses shortest-path "
         "DeltaAngleRad. Below 1 m/s groundspeed, lateral acceleration is "
         "zero. "
         "Wind is therefore included through measured ground velocity, but "
         "full "
         "NPFG wind-feasibility/path geometry is intentionally out of "
         "scope.\n\n"
      << "## 4. Validation candidate\n\n"
      << "Period=" << Format(selected.periodSec, 1)
      << " s, damping=" << Format(selected.damping, 2)
      << ", max roll=" << Format(selected.maxRollDeg, 1)
      << " deg, slew=" << Format(selected.slewDegPerSec, 1)
      << " deg/s. This probe does not modify production defaults or controller "
         "gains.\n\n"
      << "## 5. 30/120 Hz, airspeed, and step robustness\n\n"
      << "|scenario|Hz|kt|step|settling|overshoot|steady error|RMS error|max "
         "roll SP|max roll|roll SP rate|peak p|RMS r|RMS beta|max ail|max "
         "rud|roll/slew/sat fraction|sat|avg/p95/p99 tick us|\n"
      << "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:"
         "|---:|---:|:---:|---:|\n";
  for (const Run &run : robustness) {
    const Metrics &m = run.metrics;
    out << '|' << run.scenario << '|' << run.hz << '|' << run.airspeedKts << '|'
        << Format(run.stepDeg, 1) << '|' << Format(m.settlingSec) << '|'
        << Format(m.overshootDeg) << '|' << Format(m.steadyCourseErrorDeg)
        << '|' << Format(m.rmsCourseErrorDeg) << '|'
        << Format(m.maxCommandedRollDeg) << '|' << Format(m.maxActualRollDeg)
        << '|' << Format(m.peakCommandedRollRateDegPerSec) << '|'
        << Format(m.peakActualRollRateDegPerSec) << '|'
        << Format(m.rmsYawRateDegPerSec) << '|' << Format(m.rmsBetaDeg) << '|'
        << Format(m.maxAileron) << '|' << Format(m.maxRudder) << '|'
        << Format(m.rollLimitedFraction * 100.0, 1) << "%/"
        << Format(m.slewLimitedFraction * 100.0, 1) << "%/"
        << Format(m.saturationFraction * 100.0, 1) << "%|"
        << (m.saturated ? "yes" : "no") << '|' << Format(m.averageTickUsec, 1)
        << '/' << Format(m.p95TickUsec, 1) << '/' << Format(m.p99TickUsec, 1)
        << "|\n";
  }
  out << "\n## 6. Fixed-gain Roll/Yaw regression\n\n"
      << "Roll FF/P/I/D remains 1.20/1.90/0.25/0 and yaw augmentation remains "
         "P=0.8, K_beta=8, I/D/FF=0 for every row. No gain is selected or "
         "written by this regression.\n\n"
      << "|scenario|Hz|roll cmd|overshoot|settling|steady error|p max/min|RMS "
         "p error|peak/RMS r|peak/RMS beta|max ail/rud|sat|\n"
      << "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|\n";
  for (const RollRegressionMetrics &m : rollRegression) {
    out << '|' << m.scenario << '|' << m.hz << '|'
        << Format(m.commandedRollDeg, 1) << '|' << Format(m.overshootDeg) << '|'
        << Format(m.settlingSec) << '|' << Format(m.steadyErrorDeg) << '|'
        << Format(m.peakRollRateDegPerSec) << '/'
        << Format(m.minimumRollRateDegPerSec) << '|'
        << Format(m.rmsRollRateErrorDegPerSec) << '|'
        << Format(m.peakYawRateDegPerSec) << '/'
        << Format(m.rmsYawRateDegPerSec) << '|' << Format(m.peakBetaDeg) << '/'
        << Format(m.rmsBetaDeg) << '|' << Format(m.maxAileron) << '/'
        << Format(m.maxRudder) << '|' << (m.saturated ? "yes" : "no") << "|\n";
  }
  out << "\n## 7. Determinism and recommendation\n\n"
      << "Two fresh 120 Hz nominal runs produced "
      << (deterministic ? "identical metrics within 1e-9."
                        : "different metrics.")
      << " Headless tick latency is recorded above and should remain well "
         "below "
         "8333 us. GUI FPS was not forced to 120 and is not benchmarked by "
         "this "
         "headless probe. Production may use 120 Hz as the default when the "
         "full "
         "ctest suite passes; do not auto-apply any sweep-selected gain "
         "changes.\n\n"
      << "Remaining limitations: no waypoint/path geometry, no explicit wind "
         "triangle feasibility mapper, and no automated GUI-frame CPU "
         "measurement.\n";
}

double ParsePositiveCandidateValue(const char *text, std::string_view name) {
  std::size_t parsed = 0;
  const std::string valueText = text;
  const double value = std::stod(valueText, &parsed);
  if (parsed != valueText.size() || !std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(
        std::string(name) + " must be a finite positive number");
  }
  return value;
}

ProbeOptions ParseOptions(int argc, char **argv) {
  ProbeOptions options;
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      std::cout << "Usage: px4_course_tuning_probe [--output DIRECTORY] "
                   "[--candidate PERIOD_SEC DAMPING MAX_ROLL_DEG "
                   "SLEW_DEG_S] [--validation-only]\n";
      std::exit(0);
    }
    if (argument == "--output" && index + 1 < argc) {
      options.output = argv[++index];
    } else if (argument == "--candidate" && index + 4 < argc) {
      options.candidate =
          Candidate{ParsePositiveCandidateValue(argv[++index], "period"),
              ParsePositiveCandidateValue(argv[++index], "damping"),
              ParsePositiveCandidateValue(argv[++index], "max roll"),
              ParsePositiveCandidateValue(argv[++index], "slew rate")};
    } else if (argument == "--validation-only") {
      options.validationOnly = true;
    } else {
      throw std::runtime_error(
          "Unknown or incomplete argument: " + std::string(argument));
    }
  }
  if (options.validationOnly && !options.candidate.has_value()) {
    throw std::runtime_error("--validation-only requires --candidate");
  }
  return options;
}
} // namespace

int main(int argc, char **argv) {
  try {
    const ProbeOptions options = ParseOptions(argc, argv);
    const std::filesystem::path &output = options.output;
    std::filesystem::create_directories(output);
    std::vector<Run> sweep;
    if (options.validationOnly) {
      std::cout << "[1/6] nominal sweep skipped for candidate validation\n";
    } else {
      std::cout << "[1/6] nominal 120 Hz course sweep\n";
      for (double period : {8.0, 10.0, 14.0}) {
        for (double damping : {0.5, 0.7, 0.9}) {
          for (double maxRoll : {20.0, 25.0}) {
            for (double slew : {45.0, 90.0}) {
              const Candidate candidate{period, damping, maxRoll, slew};
              sweep.push_back(Execute("sweep_+10",
                  120.0,
                  80.0,
                  0.0,
                  10.0,
                  candidate,
                  false));
            }
          }
        }
      }
      std::sort(sweep.begin(),
          sweep.end(),
          [](const Run &left, const Run &right) {
            return left.metrics.score < right.metrics.score;
          });
    }
    const Candidate selected = options.candidate.has_value()
                                   ? *options.candidate
                                   : sweep.front().candidate;

    std::cout << "[2/6] top candidate telemetry\n";
    std::vector<Run> topSamples;
    if (options.candidate.has_value()) {
      topSamples.push_back(
          Execute("validation_+10", 120.0, 80.0, 0.0, 10.0, selected, true));
    } else {
      for (std::size_t index = 0;
          index < std::min<std::size_t>(3, sweep.size());
          ++index) {
        topSamples.push_back(Execute("top_+10",
            120.0,
            80.0,
            0.0,
            10.0,
            sweep[index].candidate,
            true));
      }
    }

    std::cout << "[3/6] 120 Hz nominal / 30 Hz robustness matrix\n";
    std::vector<Run> robustness;
    for (double hz : {120.0, 30.0}) {
      for (double airspeed : {60.0, 80.0, 100.0}) {
        for (double step : {-20.0, -10.0, -5.0, 5.0, 10.0, 20.0}) {
          robustness.push_back(
              Execute("course_step", hz, airspeed, 0.0, step, selected, false));
        }
      }
    }
    robustness.push_back(Execute("wrap_+179_to_-179",
        120.0,
        80.0,
        179.0,
        -179.0,
        selected,
        true));
    robustness.push_back(
        Execute("enable_+10", 120.0, 80.0, 0.0, 10.0, selected, true, true));

    std::vector<RollRegressionMetrics> rollRegression;
    if (options.validationOnly) {
      std::cout << "[4/6] fixed-gain Roll/Yaw regression skipped\n";
    } else {
      std::cout << "[4/6] fixed-gain Roll/Yaw regression\n";
      for (double hz : {120.0, 30.0}) {
        for (double step : {5.0, 10.0, -5.0}) {
          rollRegression.push_back(
              ExecuteRollRegression("roll_step", hz, step, false));
        }
        rollRegression.push_back(
            ExecuteRollRegression("direct_rate_pulse", hz, 0.0, true));
      }
    }

    std::cout << "[5/6] deterministic repeat\n";
    const Run repeatA =
        Execute("determinism_a", 120.0, 80.0, 0.0, 10.0, selected, false);
    const Run repeatB =
        Execute("determinism_b", 120.0, 80.0, 0.0, 10.0, selected, false);
    const bool deterministic =
        std::abs(repeatA.metrics.rmsCourseErrorDeg
                 - repeatB.metrics.rmsCourseErrorDeg)
            <= 1.0e-9
        && std::abs(repeatA.metrics.rmsYawRateDegPerSec
                    - repeatB.metrics.rmsYawRateDegPerSec)
               <= 1.0e-9
        && std::abs(repeatA.metrics.maxAileron - repeatB.metrics.maxAileron)
               <= 1.0e-9;

    std::cout << "[6/6] writing artifacts\n";
    std::vector<Run> allRows = sweep;
    allRows.insert(allRows.end(), robustness.begin(), robustness.end());
    WriteSweep(output / "course_sweep.csv", allRows);
    WriteSamples(output / "top_candidates_timeseries.csv", topSamples);
    if (!sweep.empty()) {
      WriteHeatmap(output / "score_heatmap.svg", sweep);
    }
    WriteReport(output / "report.md",
        selected,
        robustness,
        rollRegression,
        deterministic);
    std::cout << "Validation candidate: period="
              << Format(selected.periodSec, 1)
              << " damping=" << Format(selected.damping, 2)
              << " max_roll=" << Format(selected.maxRollDeg, 1)
              << " slew=" << Format(selected.slewDegPerSec, 1)
              << "\nReport: " << std::filesystem::absolute(output / "report.md")
              << '\n';
  } catch (const std::exception &error) {
    std::cerr << "px4_course_tuning_probe: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
