#include "common/math/Math.hpp"
#include "sim/FDMState.hpp"
#include "sim/Simulation.hpp"
#include "sim/control/FlightControlManager.hpp"
#include "sim/control/FlightControlMode.hpp"
#include "sim/gnc/autopilot/PX4Autopilot.hpp"
#include "sim/gnc/control/attitude/Px4RollController.hpp"
#include "sim/gnc/control/yaw/Px4YawRateController.hpp"

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
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
constexpr double PulseStartSec = 5.0;
constexpr double PulseEndSec = 7.0;
constexpr double RunDurationSec = 20.0;
constexpr double PulseRateDegPerSec = 5.0;
constexpr double RollFf = 1.20;
constexpr double RollP = 1.90;
constexpr double RollI = 0.25;
constexpr double RollD = 0.0;

struct YawGains {
  double p{};
  double i{};
  double d{};
  double ff{};
  double rollToYawFf{};
  double betaToYawRate{};
  double washoutTimeConstantSec{};
};

struct Sample {
  double time{};
  double pSp{};
  double p{};
  double rSp{};
  double r{};
  double beta{};
  double phi{};
  double yawError{};
  double yawP{};
  double yawI{};
  double yawD{};
  double yawFf{};
  double rollToYawFf{};
  double integrator{};
  double unscaledTorque{};
  double rawTorque{};
  double saturatedTorque{};
  double rudderCommand{};
  double rudderSurface{};
  double aileronCommand{};
  double aileronSurface{};
  double airspeedScaling{};
  bool saturated{};
  bool integratorLimited{};
};

struct Metrics {
  double pulseMeanP{};
  double pulseRmsRollError{};
  double rollOvershoot{};
  double postPeakR{};
  double postRmsR{};
  double postPeakBeta{};
  double postRmsBeta{};
  double postRmsP{};
  double pDecayRatio{};
  double oscillationPeriod{};
  double settling{};
  double maxRudderCommandDelta{};
  double maxRudderSurfaceDeltaDeg{};
  double maxRudderSlewDegPerSec{};
  double rudderVariationPerSec{};
  bool saturated{};
  double score{};
};

struct Run {
  std::string stage;
  double hz{};
  YawGains gains;
  gnc::Px4YawRateSetpointMode mode = gnc::Px4YawRateSetpointMode::DampingOnly;
  Metrics metrics;
  std::vector<Sample> samples;
};

struct BiasMetrics {
  double meanTailR{};
  double rmsTailR{};
  double finalIntegrator{};
  double limitedFraction{};
  bool saturated{};
};

struct FullRollMetrics {
  std::string profile;
  double steadyRollError{};
  double peakR{};
  double rmsR{};
  double peakBeta{};
  double rmsBeta{};
  double maxRudderDelta{};
  bool saturated{};
};

double Square(double value) { return value * value; }

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
          [](double sum, double value) { return sum + Square(value); })
      / static_cast<double>(values.size()));
}

double FiniteOr(double value, double fallback) {
  return std::isfinite(value) ? value : fallback;
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

gnc::Px4RollController &RollController(
    sim::Simulation &simulation) {
  auto *controller = Autopilot(simulation)
                         .GetController<gnc::Px4RollController>();
  if (!controller) {
    throw std::runtime_error("PX4 roll controller is missing");
  }
  return *controller;
}

Metrics Evaluate(const std::vector<Sample> &samples, double hz,
    double initialRudder, double initialSurface) {
  Metrics result;
  std::vector<double> pulseErrors;
  std::vector<double> pulseTail;
  std::vector<double> postP;
  std::vector<double> postR;
  std::vector<double> postBeta;
  std::vector<double> positivePeaks;
  std::vector<double> peakTimes;
  double previousRudder = samples.front().rudderCommand;
  double previousSurface = samples.front().rudderSurface;
  double previousP = samples.front().p;
  double lastOutside = PulseEndSec;
  bool outsideAtEnd = false;

  for (std::size_t index = 0; index < samples.size(); ++index) {
    const Sample &sample = samples[index];
    if (sample.time >= PulseEndSec - 0.5 && sample.time <= PulseEndSec) {
      pulseTail.push_back(sample.p);
    }
    if (sample.time >= PulseStartSec && sample.time <= PulseEndSec) {
      pulseErrors.push_back(sample.pSp - sample.p);
      result.rollOvershoot =
          std::max(result.rollOvershoot, sample.p - PulseRateDegPerSec);
    }
    if (sample.time >= PulseEndSec + 0.25 && sample.time <= 17.0) {
      postP.push_back(sample.p);
      postR.push_back(sample.r);
      postBeta.push_back(sample.beta);
      result.postPeakR = std::max(result.postPeakR, std::abs(sample.r));
      result.postPeakBeta =
          std::max(result.postPeakBeta, std::abs(sample.beta));
      outsideAtEnd = std::abs(sample.p) > 0.25 || std::abs(sample.r) > 0.25
                     || std::abs(sample.beta) > 0.15;
      if (outsideAtEnd) {
        lastOutside = sample.time;
      }
    }
    if (index > 0) {
      result.maxRudderSlewDegPerSec = std::max(result.maxRudderSlewDegPerSec,
          std::abs(sample.rudderSurface - previousSurface) * hz);
      if (sample.time >= PulseStartSec && sample.time <= 17.0) {
        result.rudderVariationPerSec +=
            std::abs(sample.rudderCommand - previousRudder) / 12.0;
      }
      if (index + 1 < samples.size() && sample.time >= PulseEndSec
          && sample.p > previousP && sample.p > samples[index + 1].p
          && sample.p > 0.02) {
        positivePeaks.push_back(sample.p);
        peakTimes.push_back(sample.time);
      }
    }
    previousP = sample.p;
    previousRudder = sample.rudderCommand;
    previousSurface = sample.rudderSurface;
    result.maxRudderCommandDelta = std::max(result.maxRudderCommandDelta,
        std::abs(sample.rudderCommand - initialRudder));
    result.maxRudderSurfaceDeltaDeg = std::max(result.maxRudderSurfaceDeltaDeg,
        std::abs(sample.rudderSurface - initialSurface));
    result.saturated = result.saturated || sample.saturated;
  }

  result.pulseMeanP = Mean(pulseTail);
  result.pulseRmsRollError = Rms(pulseErrors);
  result.rollOvershoot = std::max(0.0, result.rollOvershoot);
  result.postRmsP = Rms(postP);
  result.postRmsR = Rms(postR);
  result.postRmsBeta = Rms(postBeta);
  result.settling = outsideAtEnd
                        ? std::numeric_limits<double>::infinity()
                        : std::max(0.0, lastOutside - PulseEndSec + 1.0 / hz);
  if (positivePeaks.size() >= 2) {
    result.pDecayRatio = positivePeaks[1] / positivePeaks[0];
    result.oscillationPeriod = peakTimes[1] - peakTimes[0];
  }
  return result;
}

double Normalized(double value, double reference, double floor) {
  return std::min(std::abs(value) / std::max(std::abs(reference), floor), 4.0);
}

double Score(const Metrics &metrics, const Metrics &baseline) {
  const std::array costs{
      Normalized(metrics.postRmsR, baseline.postRmsR, 0.05),
      Normalized(metrics.postRmsBeta, baseline.postRmsBeta, 0.05),
      Normalized(metrics.postRmsP, baseline.postRmsP, 0.05),
      Normalized(FiniteOr(metrics.settling, 12.0),
          FiniteOr(baseline.settling, 12.0),
          1.0),
      Normalized(metrics.maxRudderCommandDelta, 0.12, 0.12),
      Normalized(metrics.maxRudderSlewDegPerSec, 20.0, 20.0),
      Normalized(metrics.pulseRmsRollError, baseline.pulseRmsRollError, 0.10),
  };
  return std::accumulate(costs.begin(), costs.end(), 0.0) / costs.size()
         + (metrics.saturated ? 2.0 : 0.0);
}

class Harness {
public:
  explicit Harness(double hz) : hz_(hz) {}

  Run DirectPulse(std::string stage, YawGains gains,
      gnc::Px4YawRateSetpointMode mode, bool retainSamples = false) {
    const auto [initialRudder, initialSurface] = Reset(gains, mode, true);
    auto &roll = RollController(*simulation_);
    auto rollSettings = roll.GetSettings();
    std::vector<Sample> samples;
    samples.reserve(static_cast<std::size_t>(RunDurationSec * hz_));
    double previousCommand = std::numeric_limits<double>::quiet_NaN();
    for (int tick = 0; tick < std::lround(RunDurationSec * hz_); ++tick) {
      const double time = tick / hz_;
      const double command = time >= PulseStartSec && time < PulseEndSec
                                 ? PulseRateDegPerSec
                                 : 0.0;
      if (command != previousCommand) {
        previousCommand = command;
        rollSettings.directRollRateCommandRadPerSec = math::DegToRad(command);
        roll.SetSettings(rollSettings);
      }
      Tick();
      samples.push_back(Capture());
    }
    Run run{std::move(stage), hz_, gains, mode};
    run.metrics = Evaluate(samples, hz_, initialRudder, initialSurface);
    if (retainSamples) {
      run.samples = std::move(samples);
    }
    return run;
  }

  BiasMetrics Bias(YawGains gains) {
    Reset(gains, gnc::Px4YawRateSetpointMode::DampingOnly, true);
    auto settings = Autopilot(*simulation_).GetYawRateSettings();
    const double initialTrim = settings.trimRudderCommand;
    std::vector<Sample> samples;
    for (int tick = 0; tick < std::lround(30.0 * hz_); ++tick) {
      if (tick == std::lround(PulseStartSec * hz_)) {
        settings.trimRudderCommand = initialTrim + 0.02;
        Autopilot(*simulation_).SetYawRateSettings(settings);
      }
      Tick();
      samples.push_back(Capture());
    }
    BiasMetrics result;
    std::vector<double> tail;
    std::size_t limited = 0;
    for (const Sample &sample : samples) {
      limited += sample.integratorLimited ? 1u : 0u;
      result.saturated = result.saturated || sample.saturated;
      if (sample.time >= 25.0) {
        tail.push_back(sample.r);
      }
    }
    result.meanTailR = Mean(tail);
    result.rmsTailR = Rms(tail);
    result.finalIntegrator = samples.back().integrator;
    result.limitedFraction =
        static_cast<double>(limited) / static_cast<double>(samples.size());
    return result;
  }

  FullRollMetrics FullRoll(std::string profile, YawGains gains) {
    const auto [initialRudder, unused] =
        Reset(gains, gnc::Px4YawRateSetpointMode::CoordinatedTurn, false);
    (void)unused;
    auto &autopilot = Autopilot(*simulation_);
    const double initialRoll =
        simulation_->GetAircraft().GetProperties().Roll().Rad();
    std::vector<double> tailError;
    std::vector<double> postR;
    std::vector<double> postBeta;
    FullRollMetrics result;
    result.profile = std::move(profile);
    for (int tick = 0; tick < std::lround(30.0 * hz_); ++tick) {
      const double time = tick / hz_;
      double offset = 0.0;
      if (result.profile.ends_with("+5")) {
        offset = time >= 5.0 ? 5.0 : 0.0;
      } else if (result.profile.ends_with("+10")) {
        offset = time >= 5.0 ? 10.0 : 0.0;
      } else if (result.profile.ends_with("-5")) {
        offset = time >= 5.0 ? -5.0 : 0.0;
      } else if (time >= 5.0) {
        offset = (static_cast<int>((time - 5.0) / 5.0) % 2 == 0) ? 5.0 : -5.0;
      }
      autopilot.SetTargetRollRad(initialRoll + math::DegToRad(offset));
      Tick();
      const Sample sample = Capture();
      if (time >= 25.0) {
        tailError.push_back(
            math::RadToDeg(autopilot.GetTargetRollRad()) - sample.phi);
      }
      if (time >= 7.0) {
        postR.push_back(sample.r);
        postBeta.push_back(sample.beta);
      }
      result.peakR = std::max(result.peakR, std::abs(sample.r));
      result.peakBeta = std::max(result.peakBeta, std::abs(sample.beta));
      result.maxRudderDelta = std::max(result.maxRudderDelta,
          std::abs(sample.rudderCommand - initialRudder));
      result.saturated = result.saturated || sample.saturated;
    }
    result.steadyRollError = Mean(tailError);
    result.rmsR = Rms(postR);
    result.rmsBeta = Rms(postBeta);
    return result;
  }

private:
  std::pair<double, double> Reset(YawGains gains,
      gnc::Px4YawRateSetpointMode mode, bool directRoll) {
    simulation_ = std::make_unique<sim::Simulation>(
        std::make_unique<gnc::PX4Autopilot>());
    if (!simulation_->Initialize(opts::simulation::AircraftName, hz_)) {
      throw std::runtime_error("Failed to initialize tuning simulation");
    }
    auto &autopilot = Autopilot(*simulation_);
    auto rollSettings = autopilot.GetRollHoldSettings();
    rollSettings.rateFeedForwardGain = RollFf;
    rollSettings.rateProportionalGain = RollP;
    rollSettings.rateIntegralGain = RollI;
    rollSettings.rateDerivativeGain = RollD;
    rollSettings.directRollRateTestEnabled = directRoll;
    rollSettings.directRollRateCommandRadPerSec = 0.0;
    autopilot.SetRollHoldSettings(rollSettings);

    auto yawSettings = autopilot.GetYawRateSettings();
    yawSettings.setpointMode = mode;
    yawSettings.rateProportionalGain = gains.p;
    yawSettings.rateIntegralGain = gains.i;
    yawSettings.rateDerivativeGain = gains.d;
    yawSettings.rateFeedForwardGain = gains.ff;
    yawSettings.rollToYawFeedForwardGain = gains.rollToYawFf;
    yawSettings.sideslipToYawRateGain = gains.betaToYawRate;
    yawSettings.yawRateWashoutTimeConstantSec = gains.washoutTimeConstantSec;
    autopilot.SetYawRateSettings(yawSettings);
    autopilot.SetTargetRollRad(
        simulation_->GetAircraft().GetProperties().Roll().Rad());
    autopilot.SetRollHoldEnabled(true);
    autopilot.SetYawRateControlEnabled(true);
    Manager(*simulation_).SetMode(control::FlightControlMode::Autopilot);
    const sim::FDMState state = simulation_->GetAircraft().ExtractFDMState(
        sim::FDMStateFlags::Controls);
    return {yawSettings.trimRudderCommand,
        math::RadToDeg(state.controls.rudderPositionRad)};
  }

  void Tick() {
    if (!simulation_->Tick()) {
      throw std::runtime_error("Simulation tick failed");
    }
  }

  Sample Capture() const {
    const auto &autopilot = Autopilot(*simulation_);
    const auto &roll = autopilot.GetRollHoldDiagnostics();
    const auto &yaw = autopilot.GetYawRateDiagnostics();
    const sim::FDMState state = simulation_->GetAircraft().ExtractFDMState(
        sim::FDMStateFlags::State | sim::FDMStateFlags::Controls);
    return {
        simulation_->GetTime(),
        math::RadToDeg(roll.bodyRateSetpointRadPerSec),
        math::RadToDeg(state.state.bodyAngularRatesRadPerSec[0]),
        math::RadToDeg(yaw.bodyRateSetpointRadPerSec),
        math::RadToDeg(state.state.bodyAngularRatesRadPerSec[2]),
        simulation_->GetAircraft().GetProperties().Beta().Deg(),
        math::RadToDeg(state.state.attitudeRad[0]),
        math::RadToDeg(yaw.bodyRateErrorRadPerSec),
        yaw.rateProportionalTerm,
        yaw.rateIntegralTerm,
        yaw.rateDerivativeTerm,
        yaw.rateFeedForwardTerm,
        yaw.rollToYawFeedForwardTerm,
        yaw.rateIntegrator,
        yaw.unscaledTorqueCommand,
        yaw.rawTorqueCommand,
        yaw.yawTorqueCommand,
        yaw.rudderCommand,
        math::RadToDeg(state.controls.rudderPositionRad),
        roll.aileronCommand,
        math::RadToDeg(0.5
                       * (state.controls.leftAileronPositionRad
                           - state.controls.rightAileronPositionRad)),
        yaw.airspeedScaling,
        yaw.positiveSaturation || yaw.negativeSaturation,
        yaw.integratorLimited,
    };
  }

  double hz_;
  std::unique_ptr<sim::Simulation> simulation_;
};

void AssignScores(std::vector<Run> &runs, const Metrics &baseline) {
  for (Run &run : runs) {
    run.metrics.score = Score(run.metrics, baseline);
  }
}

bool Acceptable(const Run &run, const Metrics &baseline) {
  return !run.metrics.saturated
         && run.metrics.pulseRmsRollError
                <= std::max(1.15 * baseline.pulseRmsRollError, 0.20)
         && run.metrics.maxRudderCommandDelta <= 0.35
         && run.metrics.maxRudderSlewDegPerSec <= 45.0;
}

bool JointDampingImprovement(const Run &run, const Metrics &baseline) {
  return Acceptable(run, baseline) && run.gains.p > 0.0
         && run.metrics.postRmsR <= 0.98 * baseline.postRmsR
         && run.metrics.postRmsBeta <= 0.98 * baseline.postRmsBeta
         && run.metrics.postRmsP <= 1.05 * baseline.postRmsP;
}

std::vector<Run> Ranked(std::vector<Run> runs, const Metrics &baseline,
    std::size_t count = 3) {
  AssignScores(runs, baseline);
  std::stable_sort(runs.begin(),
      runs.end(),
      [&](const Run &left, const Run &right) {
        if (Acceptable(left, baseline) != Acceptable(right, baseline)) {
          return Acceptable(left, baseline);
        }
        return left.metrics.score < right.metrics.score;
      });
  if (runs.size() > count) {
    runs.resize(count);
  }
  return runs;
}

std::vector<Run> RankedAugmented(std::vector<Run> runs, const Metrics &baseline,
    std::size_t count = 3) {
  AssignScores(runs, baseline);
  std::stable_sort(runs.begin(),
      runs.end(),
      [&](const Run &left, const Run &right) {
        if (JointDampingImprovement(left, baseline)
            != JointDampingImprovement(right, baseline)) {
          return JointDampingImprovement(left, baseline);
        }
        return left.metrics.score < right.metrics.score;
      });
  if (runs.size() > count) {
    runs.resize(count);
  }
  return runs;
}

void WriteMetrics(const std::filesystem::path &path,
    const std::vector<Run> &runs, const Metrics &baseline) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Cannot write " + path.string());
  }
  output << std::setprecision(10)
         << "stage,hz,mode,yaw_p,yaw_i,yaw_d,yaw_ff,roll_to_yaw_ff,"
            "beta_to_yaw_rate,washout_tc_s,"
            "pulse_mean_p_deg_s,pulse_rms_roll_error_deg_s,"
            "roll_overshoot_deg_s,post_peak_r_deg_s,post_rms_r_deg_s,"
            "post_peak_beta_deg,post_rms_beta_deg,post_rms_p_deg_s,"
            "p_decay_ratio,oscillation_period_s,settling_s,"
            "max_rudder_command_delta,max_rudder_surface_delta_deg,"
            "max_rudder_slew_deg_s,rudder_variation_per_s,saturated,score,"
            "acceptable\n";
  for (const Run &run : runs) {
    const Metrics &m = run.metrics;
    output << run.stage << ',' << run.hz << ','
           << (run.mode == gnc::Px4YawRateSetpointMode::DampingOnly
                      ? "damping"
                      : "coordinated")
           << ',' << run.gains.p << ',' << run.gains.i << ',' << run.gains.d
           << ',' << run.gains.ff << ',' << run.gains.rollToYawFf << ','
           << run.gains.betaToYawRate << ',' << run.gains.washoutTimeConstantSec
           << ',' << m.pulseMeanP << ',' << m.pulseRmsRollError << ','
           << m.rollOvershoot << ',' << m.postPeakR << ',' << m.postRmsR << ','
           << m.postPeakBeta << ',' << m.postRmsBeta << ',' << m.postRmsP << ','
           << m.pDecayRatio << ',' << m.oscillationPeriod << ','
           << FiniteOr(m.settling, -1.0) << ',' << m.maxRudderCommandDelta
           << ',' << m.maxRudderSurfaceDeltaDeg << ','
           << m.maxRudderSlewDegPerSec << ',' << m.rudderVariationPerSec << ','
           << (m.saturated ? 1 : 0) << ',' << m.score << ','
           << (Acceptable(run, baseline) ? 1 : 0) << '\n';
  }
}

void WriteSamples(const std::filesystem::path &path,
    const std::vector<Run> &runs) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Cannot write " + path.string());
  }
  output << std::setprecision(10)
         << "candidate,stage,hz,yaw_p,yaw_i,yaw_d,yaw_ff,roll_to_yaw_ff,"
            "beta_to_yaw_rate,washout_tc_s,"
            "time_s,p_sp_deg_s,p_deg_s,r_sp_deg_s,r_deg_s,beta_deg,phi_deg,"
            "yaw_error_deg_s,yaw_p_term,yaw_i_term,yaw_d_term,yaw_ff_term,"
            "roll_to_yaw_ff_term,yaw_integrator,unscaled_torque,raw_torque,"
            "saturated_torque,rudder_command,rudder_surface_deg,"
            "aileron_command,aileron_surface_deg,airspeed_scaling,saturated,"
            "integrator_limited\n";
  for (std::size_t index = 0; index < runs.size(); ++index) {
    const Run &run = runs[index];
    for (const Sample &s : run.samples) {
      output << index + 1 << ',' << run.stage << ',' << run.hz << ','
             << run.gains.p << ',' << run.gains.i << ',' << run.gains.d << ','
             << run.gains.ff << ',' << run.gains.rollToYawFf << ','
             << run.gains.betaToYawRate << ','
             << run.gains.washoutTimeConstantSec << ',' << s.time << ','
             << s.pSp << ',' << s.p << ',' << s.rSp << ',' << s.r << ','
             << s.beta << ',' << s.phi << ',' << s.yawError << ',' << s.yawP
             << ',' << s.yawI << ',' << s.yawD << ',' << s.yawFf << ','
             << s.rollToYawFf << ',' << s.integrator << ',' << s.unscaledTorque
             << ',' << s.rawTorque << ',' << s.saturatedTorque << ','
             << s.rudderCommand << ',' << s.rudderSurface << ','
             << s.aileronCommand << ',' << s.aileronSurface << ','
             << s.airspeedScaling << ',' << (s.saturated ? 1 : 0) << ','
             << (s.integratorLimited ? 1 : 0) << '\n';
    }
  }
}

std::string Polyline(const Run &run, double Sample::*member, double minValue,
    double maxValue, double top, double height) {
  std::ostringstream points;
  const double range = std::max(maxValue - minValue, 1.0e-6);
  for (std::size_t index = 0; index < run.samples.size(); index += 3) {
    const Sample &sample = run.samples[index];
    const double x = 70.0 + 850.0 * sample.time / RunDurationSec;
    const double y =
        top + height - height * (sample.*member - minValue) / range;
    points << x << ',' << y << ' ';
  }
  return points.str();
}

void WriteTraceSvg(const std::filesystem::path &path,
    const std::vector<Run> &runs) {
  std::ofstream output(path);
  output << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"960\" "
            "height=\"720\" viewBox=\"0 0 960 720\"><rect width=\"100%\" "
            "height=\"100%\" fill=\"#111827\"/><style>text{fill:#e5e7eb;"
            "font:13px sans-serif}.grid{stroke:#374151;stroke-width:1}</style>"
         << "<text x=\"30\" y=\"28\">Top yaw-damper candidates: r / beta / "
            "rudder</text>";
  const std::array colors{"#60a5fa", "#f59e0b", "#34d399"};
  for (int panel = 0; panel < 3; ++panel) {
    const double top = 55.0 + panel * 215.0;
    output << "<rect x=\"70\" y=\"" << top
           << "\" width=\"850\" height=\"170\" fill=\"none\" "
              "class=\"grid\"/>";
    for (std::size_t index = 0; index < runs.size(); ++index) {
      const Run &run = runs[index];
      const auto member = panel == 0   ? &Sample::r
                          : panel == 1 ? &Sample::beta
                                       : &Sample::rudderCommand;
      const double bound = panel == 2 ? 0.35 : 3.0;
      output << "<polyline fill=\"none\" stroke=\"" << colors[index]
             << "\" stroke-width=\"1.5\" points=\""
             << Polyline(run, member, -bound, bound, top, 170.0) << "\"/>";
    }
    output << "<text x=\"8\" y=\"" << top + 90.0 << "\">"
           << (panel == 0      ? "r deg/s"
                  : panel == 1 ? "beta deg"
                               : "rudder")
           << "</text>";
  }
  output << "</svg>\n";
}

void WriteHeatmap(const std::filesystem::path &path,
    const std::vector<Run> &runs) {
  const double maxScore = std::max_element(runs.begin(),
      runs.end(),
      [](const Run &a, const Run &b) {
        return a.metrics.score < b.metrics.score;
      })->metrics.score;
  std::ofstream output(path);
  output << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"760\" "
            "height=\"180\"><rect width=\"100%\" height=\"100%\" "
            "fill=\"#111827\"/><style>text{fill:#e5e7eb;font:12px "
            "sans-serif}</style><text x=\"20\" y=\"24\">Yaw P sweep score "
            "(lower is better)</text>";
  for (std::size_t index = 0; index < runs.size(); ++index) {
    const double ratio = runs[index].metrics.score / std::max(maxScore, 1e-6);
    const int red = static_cast<int>(50 + 170 * ratio);
    const int green = static_cast<int>(180 - 110 * ratio);
    const double x = 20.0 + index * 100.0;
    output << "<rect x=\"" << x
           << "\" y=\"45\" width=\"88\" height=\"80\" fill=\"rgb(" << red << ','
           << green << ",80)\"/><text x=\"" << x + 8
           << "\" y=\"72\">P=" << Format(runs[index].gains.p, 3)
           << "</text><text x=\"" << x + 8
           << "\" y=\"96\">score=" << Format(runs[index].metrics.score, 2)
           << "</text>";
  }
  output << "</svg>\n";
}

void WriteBias(const std::filesystem::path &path,
    const std::vector<std::pair<double, BiasMetrics>> &runs) {
  std::ofstream output(path);
  output << "yaw_i,mean_tail_r_deg_s,rms_tail_r_deg_s,final_integrator,"
            "limited_fraction,saturated\n"
         << std::setprecision(10);
  for (const auto &[gain, metrics] : runs) {
    output << gain << ',' << metrics.meanTailR << ',' << metrics.rmsTailR << ','
           << metrics.finalIntegrator << ',' << metrics.limitedFraction << ','
           << (metrics.saturated ? 1 : 0) << '\n';
  }
}

void WriteFullRoll(const std::filesystem::path &path,
    const std::vector<FullRollMetrics> &runs) {
  std::ofstream output(path);
  output << "profile,steady_roll_error_deg,peak_r_deg_s,rms_r_deg_s,"
            "peak_beta_deg,rms_beta_deg,max_rudder_delta,saturated\n"
         << std::setprecision(10);
  for (const FullRollMetrics &run : runs) {
    output << run.profile << ',' << run.steadyRollError << ',' << run.peakR
           << ',' << run.rmsR << ',' << run.peakBeta << ',' << run.rmsBeta
           << ',' << run.maxRudderDelta << ',' << (run.saturated ? 1 : 0)
           << '\n';
  }
}

std::filesystem::path ParseOutput(int argc, char **argv) {
  std::filesystem::path output = "px4-yaw-tuning-results";
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      std::cout << "Usage: px4_yaw_tuning_probe [--output DIRECTORY]\n";
      std::exit(0);
    }
    if (argument == "--output" && index + 1 < argc) {
      output = argv[++index];
    } else {
      throw std::runtime_error(
          "Unknown or incomplete argument: " + std::string(argument));
    }
  }
  return output;
}

void WriteReport(const std::filesystem::path &path, const Run &baseline,
    const std::vector<Run> &coarse, const std::vector<Run> &top,
    const std::vector<std::pair<Run, Run>> &robust, const YawGains &selected,
    const std::vector<std::pair<double, BiasMetrics>> &iBias,
    const std::vector<Run> &iPulse, const std::vector<Run> &dRuns,
    const std::vector<Run> &rollFfRuns, const Run &coordinated,
    const std::vector<FullRollMetrics> &fullRoll) {
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("Cannot write " + path.string());
  }
  const Run &best = top.front();
  const double rReduction =
      baseline.metrics.postRmsR > 1e-9
          ? 100.0 * (1.0 - best.metrics.postRmsR / baseline.metrics.postRmsR)
          : 0.0;
  const double betaReduction =
      baseline.metrics.postRmsBeta > 1e-9
          ? 100.0
                * (1.0
                    - best.metrics.postRmsBeta / baseline.metrics.postRmsBeta)
          : 0.0;
  const double pReduction =
      baseline.metrics.postRmsP > 1e-9
          ? 100.0 * (1.0 - best.metrics.postRmsP / baseline.metrics.postRmsP)
          : 0.0;
  const bool dampingSuccess = JointDampingImprovement(best, baseline.metrics);
  out << "# PX4 Fixed-wing Yaw-rate Tuning Report\n\n"
      << "## 1. PX4 yaw controller structure research\n\n"
      << "PX4 mainline uses the common three-axis `RateControl`: `e = r_sp - "
         "r`, `u = P e + I - D r_dot + FF r_sp`. Fixed-wing rate control "
         "applies positive FF as `FF / airspeed_scaling`, then multiplies "
         "the summed controller output by `airspeed_scaling^2`, adds trim, "
         "and clamps the actuator. `FW_RLL_TO_YAW_FF` adds a fraction of "
         "the final roll output to yaw before yaw saturation. Sources: "
         "[FixedwingRateControl.cpp](https://github.com/PX4/PX4-Autopilot/"
         "blob/main/src/modules/fw_rate_control/FixedwingRateControl.cpp), "
         "[rate_control.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/"
         "src/lib/rate_control/rate_control.cpp), and "
         "[fw_rate_control_params.yaml](https://github.com/PX4/"
         "PX4-Autopilot/blob/main/src/modules/fw_rate_control/"
         "fw_rate_control_params.yaml).\n\n"
      << "The coordinated-turn setpoint follows current PX4 attitude control: "
         "`r_sp = g / V * sin(phi) * cos(theta)`, faded from 70 to 75 degrees "
         "tilt and limited by `FW_Y_RMAX`. Source: "
         "[FixedwingAttitudeControl.cpp](https://github.com/PX4/"
         "PX4-Autopilot/blob/main/src/modules/fw_att_control/"
         "FixedwingAttitudeControl.cpp). PX4 mainline defaults are YR P/I/D/"
         "FF = 0.05/0.10/0/0.30, IMAX=0.20, RLL_TO_YAW_FF=0; JSB0 does not "
         "enable or promote those defaults automatically.\n\n"
      << "## 2. JSB0 signal flow\n\n"
      << "`roll/flight condition -> yaw setpoint generator -> yaw rate "
         "controller -> rudder allocation -> JSBSim -> measured body r`. "
         "The existing Roll Hold executes first and is unchanged. The yaw "
         "controller is an independently disabled-by-default controller owned "
         "by `PX4Autopilot`. C172x has `Cndr < 0`, so PX4 positive yaw-torque "
         "sign is inverted only at the rudder-allocation boundary. Existing "
         "trim rudder is added there and never enters the rate error.\n\n"
      << "## 3. Controller equation\n\n"
      << "```text\n"
      << "r_sp = r_coord + K_beta * beta\n"
      << "e_r = r_sp - [r_coord + HPF(body_r - r_coord)]\n"
      << "s = trim_CAS / max(CAS, stall_CAS)\n"
      << "u = P*e_r + I_state - D*r_dot + (FF/s)*r_sp\n"
      << "raw_yaw_torque = u*s^2 + FW_RLL_TO_YAW_FF*final_roll_output\n"
      << "rudder = clamp(trim_rudder - raw_yaw_torque, -1, +1)\n"
      << "```\n\n"
      << "Integrator update, 400 deg/s error reduction, directional "
         "anti-windup, IMAX, and dt clamp [2,40] ms match PX4 common rate "
         "control semantics.\n\n"
      << "## 4. Direct-rate excitation\n\n"
      << "Each run constructs and fully trims a fresh simulation. Roll gains "
         "are fixed at FF/P/I/D = 1.20/1.90/0.25/0 and direct p_sp is 0 until "
         "5 s, +5 deg/s from 5-7 s, then 0. Yaw I/D/FF/RLL-FF are zero in "
         "the P sweep; damping mode fixes r_sp=0. No state crosses runs.\n\n"
      << "## 5. Yaw P sweep\n\n"
      << "The score is the equal-weight mean of baseline-normalized post-pulse "
         "RMS r, beta, p, settling, rudder command, rudder slew, and roll-rate "
         "tracking. Floors avoid a near-zero baseline dominating; saturation "
         "adds a penalty. Raw metrics remain in CSV.\n\n"
      << "|P|RMS r|RMS beta|RMS p|settling|rudder delta|slew|score|\n"
      << "|---:|---:|---:|---:|---:|---:|---:|---:|\n";
  for (const Run &run : coarse) {
    out << '|' << Format(run.gains.p, 3) << '|' << Format(run.metrics.postRmsR)
        << '|' << Format(run.metrics.postRmsBeta) << '|'
        << Format(run.metrics.postRmsP) << '|' << Format(run.metrics.settling)
        << '|' << Format(run.metrics.maxRudderCommandDelta) << '|'
        << Format(run.metrics.maxRudderSlewDegPerSec) << '|'
        << Format(run.metrics.score) << "|\n";
  }
  out << "\nThe JSB0 augmentation sweep covers P={0.05,0.1,0.2,0.4,0.8,"
         "1.6}, K_beta={-4,-2,-1,-0.5,0,0.5,1,2,4,6,8,12} rad/s per "
         "rad, followed by washout TC={0,0.25,0.5,1,2,4} s around the top "
         "joint-damping candidates.\n";
  out << "\n## 6. r/beta/p damping change\n\n"
      << "Best augmented candidate P=" << Format(best.gains.p, 3)
      << ", K_beta=" << Format(best.gains.betaToYawRate, 3)
      << " changes post-pulse RMS r by " << Format(rReduction, 1)
      << "% and RMS beta by " << Format(betaReduction, 1) << "%, and RMS p by "
      << Format(pReduction, 1)
      << "% versus yaw-off-equivalent P=0. Its p decay ratio/period are "
      << Format(best.metrics.pDecayRatio) << " / "
      << Format(best.metrics.oscillationPeriod)
      << " s. A matching r, beta, "
         "and p period is treated as Dutch-roll/lateral coupling; a new short "
         "period with growing rudder variation is controller/actuator induced."
         "\n\n"
      << (dampingSuccess
                 ? "A non-zero P candidate satisfies the joint r/beta/p "
                   "damping "
                   "guardrail.\n\n"
                 : "**No non-zero P candidate satisfies the requested joint "
                   "r/beta/p damping guardrail.** With the PX4 torque sign, "
                   "increasing P monotonically reduces r RMS but increases "
                   "beta "
                   "RMS. An explicit opposite-allocation diagnostic reverses "
                   "that tradeoff (beta falls at low P while r grows), "
                   "confirming "
                   "that this is not a hidden actuator-sign error. Pure "
                   "yaw-rate "
                   "feedback cannot damp both states at this C172x operating "
                   "point; a separately reviewed sideslip-feedback or washout "
                   "path would be required before production enablement.\n\n")
      << "## 7. Actuator behavior\n\n"
      << "The selected augmented run reaches rudder command delta "
      << Format(best.metrics.maxRudderCommandDelta) << ", actual surface delta "
      << Format(best.metrics.maxRudderSurfaceDeltaDeg)
      << " deg, and surface "
         "slew "
      << Format(best.metrics.maxRudderSlewDegPerSec) << " deg/s; saturation is "
      << (best.metrics.saturated ? "present" : "absent") << ".\n\n"
      << "## 8. Recommended yaw gains\n\n"
      << "- Experimental nominal: **P=" << Format(selected.p, 3)
      << ", I=" << Format(selected.i, 3) << ", D=" << Format(selected.d, 3)
      << ", FF=" << Format(selected.ff, 3)
      << ", K_beta=" << Format(selected.betaToYawRate, 3)
      << ", washout TC=" << Format(selected.washoutTimeConstantSec, 3)
      << " s**\n"
      << "- Production default change: **not recommended automatically**; yaw "
         "control remains disabled by default.\n"
      << "- Joint damping success: **"
      << (dampingSuccess ? "yes" : "no; retain P=0 / disabled") << "**\n"
      << "- Top augmented candidates (P/K_beta/washout): ";
  for (std::size_t index = 0; index < top.size(); ++index) {
    out << (index ? ", " : "") << Format(top[index].gains.p, 3) << '/'
        << Format(top[index].gains.betaToYawRate, 3) << '/'
        << Format(top[index].gains.washoutTimeConstantSec, 3);
  }
  out << "\n\n30/120 Hz cross-check:\n\n"
      << "|P|RMS r 30/120|RMS beta 30/120|RMS p 30/120|rudder slew 30/120|\n"
      << "|---:|---:|---:|---:|---:|\n";
  for (const auto &[run30, run120] : robust) {
    out << '|' << Format(run30.gains.p, 3) << '|'
        << Format(run30.metrics.postRmsR) << '/'
        << Format(run120.metrics.postRmsR) << '|'
        << Format(run30.metrics.postRmsBeta) << '/'
        << Format(run120.metrics.postRmsBeta) << '|'
        << Format(run30.metrics.postRmsP) << '/'
        << Format(run120.metrics.postRmsP) << '|'
        << Format(run30.metrics.maxRudderSlewDegPerSec) << '/'
        << Format(run120.metrics.maxRudderSlewDegPerSec) << "|\n";
  }
  out << "\nI bias-test rows (a +0.02 normalized rudder trim disturbance at 5 "
         "s):\n\n|I|tail mean r|tail RMS r|integrator|limited|\n"
      << "|---:|---:|---:|---:|---:|\n";
  for (const auto &[gain, metrics] : iBias) {
    out << '|' << Format(gain, 3) << '|' << Format(metrics.meanTailR) << '|'
        << Format(metrics.rmsTailR) << '|' << Format(metrics.finalIntegrator)
        << '|' << Format(metrics.limitedFraction) << "|\n";
  }
  out << "\n## 9. Roll-to-yaw feedforward\n\n"
      << "It is evaluated only after P/I/D. The sweep is preserved in "
         "`yaw_pid_sweep.csv`; the experimental selected value is "
      << Format(selected.rollToYawFf, 3)
      << ". A non-zero value is retained only when it improves damping score "
         "without degrading direct roll tracking or actuator limits.\n\n"
      << (!dampingSuccess
                 ? "Because no non-zero yaw-P foundation passed the joint "
                   "damping guardrail, FF and coordinated-turn rows are "
                   "diagnostic only and must not be interpreted as tuned "
                   "production settings.\n\n"
                 : "")
      << "## 10. Coordinated-turn mode\n\n"
      << "With coordinated r_sp, direct-pulse RMS r/beta/p are "
      << Format(coordinated.metrics.postRmsR) << " / "
      << Format(coordinated.metrics.postRmsBeta) << " / "
      << Format(coordinated.metrics.postRmsP)
      << ". This is a separate flight mode from pure damping, not folded into "
         "the yaw P selection.\n\n"
      << "## 11. 30/120 Hz robustness\n\n"
      << "The same gains are rerun with fresh trim at both rates. Since the "
         "controller clamps its integration dt to PX4's 40 ms upper bound, 30 "
         "Hz remains inside the intended interval and 120 Hz exercises the "
         "same continuous gains at a smaller dt. See the table and CSV.\n\n"
      << "## 12. Remaining slow mode\n\n"
      << "The score window targets the 3-4 s Dutch-roll response. It neither "
         "rewards nor tunes away a 20-40 s spiral-like/roll drift. Any such "
         "motion remaining after yaw damping must be analyzed separately with "
         "phi, heading/course, beta and Euler kinematics.\n\n"
      << "## 13. Production-default recommendation\n\n"
      << "Keep the feature opt-in until the full-roll profiles (+5, +10, -5, "
         "alternating +/-5) and aircraft envelope are reviewed. Results:\n\n"
      << "|profile|roll error|peak r|RMS r|peak beta|RMS beta|rudder "
         "delta|sat|\n"
      << "|---|---:|---:|---:|---:|---:|---:|:---:|\n";
  for (const FullRollMetrics &run : fullRoll) {
    out << '|' << run.profile << '|' << Format(run.steadyRollError) << '|'
        << Format(run.peakR) << '|' << Format(run.rmsR) << '|'
        << Format(run.peakBeta) << '|' << Format(run.rmsBeta) << '|'
        << Format(run.maxRudderDelta) << '|' << (run.saturated ? "yes" : "no")
        << "|\n";
  }
  out << "\nRaw sweep rows include I pulse, D, roll-to-yaw FF and coordinated "
         "comparisons. The production roll gains and controller equation were "
         "not modified.\n";
  (void)iPulse;
  (void)dRuns;
  (void)rollFfRuns;
}
} // namespace

int main(int argc, char **argv) {
  try {
    const std::filesystem::path output = ParseOutput(argc, argv);
    std::filesystem::create_directories(output);
    Harness at120Hz(120.0);

    std::cout << "[1/10] yaw P-only coarse sweep\n";
    constexpr std::array pValues{0.0, 0.05, 0.1, 0.2, 0.4, 0.8, 1.6};
    std::vector<Run> coarse;
    for (double p : pValues) {
      coarse.push_back(at120Hz.DirectPulse("p_coarse",
          {p, 0.0, 0.0, 0.0, 0.0},
          gnc::Px4YawRateSetpointMode::DampingOnly));
    }
    const Metrics baseline = coarse.front().metrics;
    AssignScores(coarse, baseline);
    const Run coarseBest = Ranked(coarse, baseline, 1).front();

    std::cout << "[2/10] P-only fine sweep around P="
              << Format(coarseBest.gains.p, 3) << '\n';
    std::vector<Run> fine;
    for (int offset = -4; offset <= 4; ++offset) {
      const double p = std::max(0.0, coarseBest.gains.p + 0.025 * offset);
      if (std::none_of(fine.begin(), fine.end(), [p](const Run &run) {
            return std::abs(run.gains.p - p) < 1.0e-9;
          })) {
        fine.push_back(at120Hz.DirectPulse("p_fine",
            {p, 0.0, 0.0, 0.0, 0.0},
            gnc::Px4YawRateSetpointMode::DampingOnly));
      }
    }
    AssignScores(fine, baseline);

    std::cout << "[3/10] yaw P x sideslip-feedback sweep\n";
    constexpr std::array augmentedPValues{0.05, 0.1, 0.2, 0.4, 0.8, 1.6};
    constexpr std::array betaValues{-4.0,
        -2.0,
        -1.0,
        -0.5,
        0.0,
        0.5,
        1.0,
        2.0,
        4.0,
        6.0,
        8.0,
        12.0};
    std::vector<Run> augmented;
    for (double p : augmentedPValues) {
      for (double betaGain : betaValues) {
        augmented.push_back(at120Hz.DirectPulse("beta_augmentation",
            {p, 0.0, 0.0, 0.0, 0.0, betaGain, 0.0},
            gnc::Px4YawRateSetpointMode::DampingOnly));
      }
    }
    AssignScores(augmented, baseline);
    std::vector<Run> augmentedTop = RankedAugmented(augmented, baseline);

    std::cout << "[4/10] yaw-rate washout sweep\n";
    constexpr std::array washoutValues{0.0, 0.25, 0.5, 1.0, 2.0, 4.0};
    std::vector<Run> washoutRuns;
    std::vector<Run> top;
    for (const Run &candidate : augmentedTop) {
      std::vector<Run> local;
      for (double washout : washoutValues) {
        YawGains gains = candidate.gains;
        gains.washoutTimeConstantSec = washout;
        local.push_back(at120Hz.DirectPulse("washout",
            gains,
            gnc::Px4YawRateSetpointMode::DampingOnly));
      }
      AssignScores(local, baseline);
      washoutRuns.insert(washoutRuns.end(), local.begin(), local.end());
      top.push_back(RankedAugmented(local, baseline, 1).front());
    }

    std::cout << "[5/10] augmented top-three 30/120 Hz robustness\n";
    Harness at30Hz(30.0);
    const Run baseline120 = at120Hz.DirectPulse("baseline_120",
        {0.0, 0.0, 0.0, 0.0, 0.0},
        gnc::Px4YawRateSetpointMode::DampingOnly);
    std::vector<std::pair<Run, Run>> robust;
    std::vector<Run> robustnessRows;
    for (const Run &candidate : top) {
      Run run30 = at30Hz.DirectPulse("robustness_30",
          candidate.gains,
          gnc::Px4YawRateSetpointMode::DampingOnly,
          true);
      Run run120 = at120Hz.DirectPulse("robustness_120",
          candidate.gains,
          gnc::Px4YawRateSetpointMode::DampingOnly);
      run30.metrics.score = Score(run30.metrics, baseline);
      run120.metrics.score = Score(run120.metrics, baseline120.metrics);
      robustnessRows.push_back(run30);
      robustnessRows.push_back(run120);
      robust.emplace_back(std::move(run30), std::move(run120));
    }
    const auto robustBest = std::min_element(robust.begin(),
        robust.end(),
        [&](const auto &left, const auto &right) {
          const bool leftJoint =
              JointDampingImprovement(left.first, baseline)
              && JointDampingImprovement(left.second, baseline120.metrics);
          const bool rightJoint =
              JointDampingImprovement(right.first, baseline)
              && JointDampingImprovement(right.second, baseline120.metrics);
          if (leftJoint != rightJoint) {
            return leftJoint;
          }
          return left.first.metrics.score + left.second.metrics.score
                 < right.first.metrics.score + right.second.metrics.score;
        });
    YawGains selected = robustBest->second.gains;

    std::cout << "[6/10] yaw I pulse and held-bias sweep\n";
    constexpr std::array iValues{0.0, 0.02, 0.05, 0.1, 0.2};
    std::vector<Run> iPulse;
    std::vector<std::pair<double, BiasMetrics>> iBias;
    for (double i : iValues) {
      YawGains gains = selected;
      gains.i = i;
      Run pulse = at120Hz.DirectPulse("i_pulse",
          gains,
          gnc::Px4YawRateSetpointMode::DampingOnly);
      pulse.metrics.score = Score(pulse.metrics, baseline);
      iPulse.push_back(pulse);
      iBias.emplace_back(i, at120Hz.Bias(gains));
    }
    const double zeroBias = std::abs(iBias.front().second.meanTailR);
    if (zeroBias > 0.05) {
      for (std::size_t index = 1; index < iBias.size(); ++index) {
        const BiasMetrics &bias = iBias[index].second;
        if (std::abs(bias.meanTailR) <= 0.2 * zeroBias
            && bias.limitedFraction < 0.05 && !bias.saturated
            && iPulse[index].metrics.score
                   <= 1.10 * iPulse.front().metrics.score) {
          selected.i = iBias[index].first;
          break;
        }
      }
    }

    std::cout << "[7/10] yaw D benefit and roll-to-yaw FF sweeps\n";
    constexpr std::array dValues{0.0, 0.005, 0.01, 0.02, 0.04};
    std::vector<Run> dRuns;
    for (double d : dValues) {
      YawGains gains = selected;
      gains.d = d;
      Run run = at120Hz.DirectPulse("d_sweep",
          gains,
          gnc::Px4YawRateSetpointMode::DampingOnly);
      run.metrics.score = Score(run.metrics, baseline);
      dRuns.push_back(run);
    }
    const Run &dZero = dRuns.front();
    for (std::size_t index = 1; index < dRuns.size(); ++index) {
      const Run &run = dRuns[index];
      if (run.metrics.score <= 0.95 * dZero.metrics.score
          && run.metrics.rudderVariationPerSec
                 <= 1.10 * dZero.metrics.rudderVariationPerSec
          && run.metrics.maxRudderSlewDegPerSec
                 <= 1.10 * dZero.metrics.maxRudderSlewDegPerSec
          && !run.metrics.saturated) {
        selected.d = run.gains.d;
        break;
      }
    }
    constexpr std::array rollFfValues{0.0, 0.02, 0.05, 0.1, 0.2};
    std::vector<Run> rollFfRuns;
    for (double value : rollFfValues) {
      YawGains gains = selected;
      gains.rollToYawFf = value;
      Run run = at120Hz.DirectPulse("roll_to_yaw_ff",
          gains,
          gnc::Px4YawRateSetpointMode::DampingOnly);
      run.metrics.score = Score(run.metrics, baseline);
      rollFfRuns.push_back(run);
    }
    const auto bestRollFf = std::min_element(rollFfRuns.begin(),
        rollFfRuns.end(),
        [](const Run &left, const Run &right) {
          return left.metrics.score < right.metrics.score;
        });
    if (bestRollFf->metrics.score <= 0.95 * rollFfRuns.front().metrics.score
        && !bestRollFf->metrics.saturated) {
      selected.rollToYawFf = bestRollFf->gains.rollToYawFf;
    }

    std::cout << "[8/10] coordinated-turn comparison\n";
    Run coordinated = at120Hz.DirectPulse("coordinated_turn",
        selected,
        gnc::Px4YawRateSetpointMode::CoordinatedTurn,
        true);
    coordinated.metrics.score = Score(coordinated.metrics, baseline);

    std::cout << "[9/10] yaw-off vs augmented full Roll Hold profiles\n";
    std::vector<FullRollMetrics> fullRoll;
    for (const std::string profile : {"+5", "+10", "-5", "alternating"}) {
      FullRollMetrics yawOff = at120Hz.FullRoll("yaw_off_" + profile, {});
      FullRollMetrics augmentedRun =
          at120Hz.FullRoll("augmented_" + profile, selected);
      fullRoll.push_back(std::move(yawOff));
      fullRoll.push_back(std::move(augmentedRun));
    }

    std::cout << "[10/10] writing reproducible artifacts\n";
    std::vector<Run> allPid = iPulse;
    allPid.insert(allPid.end(), dRuns.begin(), dRuns.end());
    allPid.insert(allPid.end(), rollFfRuns.begin(), rollFfRuns.end());
    allPid.push_back(coordinated);
    WriteMetrics(output / "yaw_p_sweep.csv", coarse, baseline);
    WriteMetrics(output / "yaw_p_fine_sweep.csv", fine, baseline);
    WriteMetrics(output / "yaw_beta_sweep.csv", augmented, baseline);
    WriteMetrics(output / "yaw_washout_sweep.csv", washoutRuns, baseline);
    WriteMetrics(output / "yaw_pid_sweep.csv", allPid, baseline);
    WriteMetrics(output / "timestep_robustness.csv", robustnessRows, baseline);
    WriteSamples(output / "top_candidates_timeseries.csv",
        {robust[0].second, robust[1].second, robust[2].second, coordinated});
    WriteHeatmap(output / "score_heatmap.svg", coarse);
    WriteTraceSvg(output / "top_candidates_timeseries.svg",
        {robust[0].second, robust[1].second, robust[2].second});
    WriteBias(output / "yaw_i_bias_sweep.csv", iBias);
    WriteFullRoll(output / "full_roll_hold.csv", fullRoll);
    WriteReport(output / "report.md",
        coarse.front(),
        coarse,
        top,
        robust,
        selected,
        iBias,
        iPulse,
        dRuns,
        rollFfRuns,
        coordinated,
        fullRoll);

    std::cout << "Completed experimental candidate: P=" << Format(selected.p, 3)
              << " I=" << Format(selected.i, 3)
              << " D=" << Format(selected.d, 3)
              << " FF=" << Format(selected.ff, 3)
              << " RLL_TO_YAW_FF=" << Format(selected.rollToYawFf, 3)
              << " BETA_TO_YAW_RATE=" << Format(selected.betaToYawRate, 3)
              << " WASHOUT_TC=" << Format(selected.washoutTimeConstantSec, 3)
              << "\nReport: " << std::filesystem::absolute(output / "report.md")
              << '\n';
  } catch (const std::exception &error) {
    std::cerr << "px4_yaw_tuning_probe: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
