#include "common/math/Math.hpp"
#include "sim/FDMState.hpp"
#include "sim/Simulation.hpp"
#include "sim/control/FlightControlManager.hpp"
#include "sim/control/FlightControlMode.hpp"
#include "sim/gnc/autopilot/PX4Autopilot.hpp"
#include "sim/gnc/control/attitude/Px4RollController.hpp"

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
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
constexpr double PulseStartSec = 5.0;
constexpr double PulseEndSec = 7.0;
constexpr double PulseRateDegPerSec = 5.0;
constexpr double RateRunDurationSec = 20.0;
constexpr double BiasRunDurationSec = 35.0;
constexpr double FullRollDurationSec = 30.0;
constexpr double BiasCommand = 0.03;

struct Gains {
  double ff{};
  double p{};
  double i{};
  double d{};
};

struct Sample {
  double time{};
  double pSp{};
  double p{};
  double phi{};
  double q{};
  double r{};
  double beta{};
  double rateError{};
  double pTerm{};
  double iTerm{};
  double dTerm{};
  double ffTerm{};
  double integrator{};
  double unscaledTorque{};
  double rawTorque{};
  double saturatedTorque{};
  double aileron{};
  double surface{};
  double reconstructedPhiDot{};
  double airspeedScaling{1.0};
  bool saturated{};
  bool integratorLimited{};
};

struct Metrics {
  double meanPulseP{};
  double steadyError{};
  double rmsError{};
  double maxError{};
  double positivePeak{};
  double overshoot{};
  double firstUndershoot{};
  double secondRebound{};
  double decayRatio{};
  double settling{};
  double postRmsP{};
  double maxCommand{};
  double maxCommandDelta{};
  double maxSurface{};
  double maxSurfaceDelta{};
  double maxSurfaceSlew{};
  double commandVariation{};
  bool saturated{};
  double peakR{};
  double peakBeta{};
  double postRmsR{};
  double postRmsBeta{};
  double score{};
};

struct RateRun {
  std::string stage;
  double hz{};
  Gains gains;
  Metrics metrics;
  std::vector<Sample> samples;
};

struct BiasMetrics {
  double meanTailP{};
  double steadyError{};
  double tailRmsP{};
  double finalIntegrator{};
  double maxIntegrator{};
  double limitedFraction{};
  double meanAileronBias{};
  bool saturated{};
};

struct IRun {
  double gain{};
  RateRun pulse;
  BiasMetrics bias;
  double score{};
  bool acceptable{};
};

struct DRun {
  double gain{};
  RateRun pulse;
  bool clearBenefit{};
};

struct FullRollRun {
  Gains gains;
  double hz{};
  double timeConstantSec{};
  std::vector<Sample> samples;
  double meanFinalRoll{};
  double steadyError{};
  double overshoot{};
  double settling{};
  double tailRange{};
  double maxP{};
  double maxCommand{};
  double peakR{};
  double peakBeta{};
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
  auto *manager = simulation.GetComponent<control::FlightControlManager>();
  if (!manager) {
    throw std::runtime_error("FlightControlManager is missing");
  }
  return *manager;
}

gnc::PX4Autopilot &Autopilot(sim::Simulation &simulation) {
  return dynamic_cast<gnc::PX4Autopilot &>(Manager(simulation).GetAutopilot());
}

gnc::Px4RollController &Controller(sim::Simulation &simulation) {
  auto *controller = Autopilot(simulation)
                         .GetController<gnc::Px4RollController>();
  if (!controller) {
    throw std::runtime_error("PX4 Roll Hold controller is missing");
  }
  return *controller;
}

double Score(const Metrics &metrics) {
  const auto cost = [](double value, double scale) {
    return std::min(std::abs(value) / scale, 5.0);
  };
  const double actuator = 0.5
                          * (cost(metrics.maxCommandDelta, 0.40)
                              + cost(metrics.maxSurfaceSlew, 90.0));
  const std::array costs{
      cost(metrics.steadyError, 0.50),
      cost(metrics.overshoot, 1.00),
      cost(metrics.firstUndershoot, 0.75),
      cost(FiniteOr(metrics.settling, 8.0), 3.00),
      cost(metrics.postRmsP, 0.25),
      actuator,
      cost(metrics.postRmsR, 0.50),
      cost(metrics.postRmsBeta, 0.50),
  };
  return std::accumulate(costs.begin(), costs.end(), 0.0) / costs.size()
         + (metrics.saturated ? 2.0 : 0.0);
}

bool Acceptable(const Metrics &metrics) {
  return std::abs(metrics.steadyError) <= 0.25 && metrics.overshoot <= 1.75
         && std::abs(metrics.firstUndershoot) <= 1.25 && metrics.settling <= 5.0
         && metrics.maxCommandDelta <= 0.50 && !metrics.saturated;
}

std::vector<double> FilterP(const std::vector<Sample> &samples, double hz) {
  const std::size_t window = static_cast<std::size_t>(
      std::max(3, static_cast<int>(std::round(0.1 * hz))));
  std::vector<double> filtered(samples.size());
  double sum = 0.0;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    sum += samples[index].p;
    if (index >= window) {
      sum -= samples[index - window].p;
    }
    filtered[index] = sum / static_cast<double>(std::min(index + 1, window));
  }
  return filtered;
}

Metrics EvaluateRate(const std::vector<Sample> &samples, double hz,
    double initialSurface) {
  const std::vector<double> filtered = FilterP(samples, hz);
  std::vector<double> pulseTail;
  std::vector<double> errors;
  std::vector<double> postP;
  std::vector<double> postR;
  std::vector<double> postBeta;
  Metrics result;
  result.positivePeak = -std::numeric_limits<double>::infinity();
  double lastOutside = PulseEndSec;
  bool outsideAtEnd = false;
  double previousCommand = samples.front().aileron;
  double previousSurface = samples.front().surface;

  for (std::size_t index = 0; index < samples.size(); ++index) {
    const Sample &sample = samples[index];
    if (sample.time >= PulseEndSec - 0.5 && sample.time <= PulseEndSec) {
      pulseTail.push_back(sample.p);
    }
    if (sample.time >= PulseStartSec && sample.time <= 15.0) {
      errors.push_back(sample.pSp - sample.p);
    }
    if (sample.time >= PulseStartSec && sample.time <= PulseEndSec + 0.5) {
      result.positivePeak = std::max(result.positivePeak, filtered[index]);
    }
    if (sample.time >= PulseEndSec && sample.time <= 15.0) {
      outsideAtEnd = std::abs(filtered[index]) > 0.25;
      if (outsideAtEnd) {
        lastOutside = sample.time;
      }
    }
    if (sample.time >= PulseEndSec + 1.0 && sample.time <= 15.0) {
      postP.push_back(sample.p);
      postR.push_back(sample.r);
      postBeta.push_back(sample.beta);
    }
    result.maxCommand = std::max(result.maxCommand, std::abs(sample.aileron));
    result.maxCommandDelta = std::max(result.maxCommandDelta,
        std::abs(sample.aileron - samples.front().aileron));
    result.maxSurface = std::max(result.maxSurface, std::abs(sample.surface));
    result.maxSurfaceDelta = std::max(result.maxSurfaceDelta,
        std::abs(sample.surface - initialSurface));
    if (index > 0) {
      result.maxSurfaceSlew = std::max(result.maxSurfaceSlew,
          std::abs(sample.surface - previousSurface) * hz);
      if (sample.time >= PulseStartSec && sample.time <= 15.0) {
        result.commandVariation += std::abs(sample.aileron - previousCommand);
      }
    }
    previousCommand = sample.aileron;
    previousSurface = sample.surface;
    result.saturated = result.saturated || sample.saturated;
    result.peakR = std::max(result.peakR, std::abs(sample.r));
    result.peakBeta = std::max(result.peakBeta, std::abs(sample.beta));
  }

  result.meanPulseP = Mean(pulseTail);
  result.steadyError = PulseRateDegPerSec - result.meanPulseP;
  result.rmsError = Rms(errors);
  for (double error : errors) {
    result.maxError = std::max(result.maxError, std::abs(error));
  }
  result.overshoot = std::max(0.0, result.positivePeak - PulseRateDegPerSec);
  result.postRmsP = Rms(postP);
  result.postRmsR = Rms(postR);
  result.postRmsBeta = Rms(postBeta);
  result.commandVariation /= 10.0;
  result.settling = outsideAtEnd
                        ? std::numeric_limits<double>::infinity()
                        : std::max(0.0, lastOutside - PulseEndSec + 1.0 / hz);

  std::optional<std::size_t> firstMinimum;
  for (std::size_t index = 1; index + 1 < samples.size(); ++index) {
    if (samples[index].time >= PulseEndSec
        && filtered[index] <= filtered[index - 1]
        && filtered[index] < filtered[index + 1] && filtered[index] < 0.0) {
      firstMinimum = index;
      result.firstUndershoot = filtered[index];
      break;
    }
  }
  if (firstMinimum) {
    for (std::size_t index = *firstMinimum + 1; index + 1 < samples.size();
        ++index) {
      if (filtered[index] >= filtered[index - 1]
          && filtered[index] > filtered[index + 1] && filtered[index] > 0.0) {
        result.secondRebound = filtered[index];
        break;
      }
    }
  }
  if (std::abs(result.firstUndershoot) > 1e-9) {
    result.decayRatio = result.secondRebound / std::abs(result.firstUndershoot);
  }
  result.score = Score(result);
  return result;
}

class Harness {
public:
  explicit Harness(double hz, double timeConstantSec = 0.4)
      : hz_(hz), timeConstantSec_(timeConstantSec) {}

  RateRun DirectPulse(std::string stage, Gains gains,
      bool keepSamples = false) {
    const double initialSurface = Reset(gains, true);
    auto &controller = Controller(*simulation_);
    auto settings = controller.GetSettings();
    std::vector<Sample> samples;
    samples.reserve(static_cast<std::size_t>(RateRunDurationSec * hz_));
    double previousCommand = std::numeric_limits<double>::quiet_NaN();
    for (int tick = 0; tick < std::lround(RateRunDurationSec * hz_); ++tick) {
      const double time = tick / hz_;
      const double command = time >= PulseStartSec && time < PulseEndSec
                                 ? PulseRateDegPerSec
                                 : 0.0;
      if (command != previousCommand) {
        previousCommand = command;
        settings.directRollRateCommandRadPerSec = math::DegToRad(command);
        controller.SetSettings(settings);
      }
      Tick();
      samples.push_back(Capture());
    }
    RateRun run{std::move(stage),
        hz_,
        gains,
        EvaluateRate(samples, hz_, initialSurface),
        {}};
    if (keepSamples) {
      run.samples = std::move(samples);
    }
    return run;
  }

  BiasMetrics Bias(Gains gains) {
    Reset(gains, true);
    auto &controller = Controller(*simulation_);
    auto settings = controller.GetSettings();
    const double trimCommand = settings.trimRollCommand;
    std::vector<Sample> samples;
    bool biasApplied = false;
    for (int tick = 0; tick < std::lround(BiasRunDurationSec * hz_); ++tick) {
      if (!biasApplied && tick / hz_ >= PulseStartSec) {
        settings.trimRollCommand = trimCommand + BiasCommand;
        controller.SetSettings(settings);
        biasApplied = true;
      }
      Tick();
      samples.push_back(Capture());
    }
    std::vector<double> tailP;
    std::vector<double> tailCommand;
    BiasMetrics result;
    std::size_t limited = 0;
    for (const Sample &sample : samples) {
      result.maxIntegrator =
          std::max(result.maxIntegrator, std::abs(sample.integrator));
      limited += sample.integratorLimited ? 1u : 0u;
      result.saturated = result.saturated || sample.saturated;
      if (sample.time >= BiasRunDurationSec - 5.0) {
        tailP.push_back(sample.p);
        tailCommand.push_back(sample.aileron - trimCommand);
      }
    }
    result.meanTailP = Mean(tailP);
    result.steadyError = -result.meanTailP;
    result.tailRmsP = Rms(tailP);
    result.finalIntegrator = samples.back().integrator;
    result.limitedFraction = static_cast<double>(limited) / samples.size();
    result.meanAileronBias = Mean(tailCommand);
    return result;
  }

  FullRollRun FullRoll(Gains gains) {
    Reset(gains, false);
    auto &autopilot = Autopilot(*simulation_);
    const double initialRoll =
        simulation_->GetAircraft().GetProperties().Roll().Rad();
    autopilot.SetTargetRollRad(initialRoll);
    std::vector<Sample> samples;
    for (int tick = 0; tick < std::lround(FullRollDurationSec * hz_); ++tick) {
      if (tick / hz_ >= PulseStartSec) {
        autopilot.SetTargetRollRad(initialRoll + math::DegToRad(5.0));
      }
      Tick();
      samples.push_back(Capture());
    }
    FullRollRun result;
    result.gains = gains;
    result.hz = hz_;
    result.timeConstantSec = timeConstantSec_;
    result.samples = samples;
    const double target = math::RadToDeg(initialRoll) + 5.0;
    std::vector<double> tail;
    double maximumRoll = -std::numeric_limits<double>::infinity();
    double lastOutside = PulseStartSec;
    bool outsideAtEnd = false;
    for (const Sample &sample : samples) {
      if (sample.time >= PulseStartSec) {
        maximumRoll = std::max(maximumRoll, sample.phi);
        outsideAtEnd = std::abs(target - sample.phi) > 0.5;
        if (outsideAtEnd) {
          lastOutside = sample.time;
        }
      }
      if (sample.time >= FullRollDurationSec - 5.0) {
        tail.push_back(sample.phi);
      }
      result.maxP = std::max(result.maxP, std::abs(sample.p));
      result.maxCommand = std::max(result.maxCommand, std::abs(sample.aileron));
      result.peakR = std::max(result.peakR, std::abs(sample.r));
      result.peakBeta = std::max(result.peakBeta, std::abs(sample.beta));
      result.saturated = result.saturated || sample.saturated;
    }
    result.meanFinalRoll = Mean(tail);
    result.steadyError = target - result.meanFinalRoll;
    result.overshoot = std::max(0.0, maximumRoll - target);
    result.settling = outsideAtEnd ? std::numeric_limits<double>::infinity()
                                   : lastOutside - PulseStartSec + 1.0 / hz_;
    const auto [minimum, maximum] =
        std::minmax_element(tail.begin(), tail.end());
    result.tailRange = *maximum - *minimum;
    return result;
  }

private:
  double Reset(Gains gains, bool direct) {
    simulation_ = std::make_unique<sim::Simulation>(
        std::make_unique<gnc::PX4Autopilot>());
    sim::SimulationConfig config;
    config.simulationHz = hz_;
    if (!simulation_->Initialize(config)) {
      throw std::runtime_error(
          "Failed to initialize a fresh tuning simulation");
    }
    auto &autopilot = Autopilot(*simulation_);
    auto &controller = Controller(*simulation_);
    auto settings = controller.GetSettings();
    settings.timeConstantSec = timeConstantSec_;
    settings.rateFeedForwardGain = gains.ff;
    settings.rateProportionalGain = gains.p;
    settings.rateIntegralGain = gains.i;
    settings.rateDerivativeGain = gains.d;
    settings.directRollRateTestEnabled = direct;
    settings.directRollRateCommandRadPerSec = 0.0;
    controller.SetSettings(settings);
    autopilot.SetTargetRollRad(
        simulation_->GetAircraft().GetProperties().Roll().Rad());
    autopilot.SetRollHoldEnabled(true);
    Manager(*simulation_).SetMode(control::FlightControlMode::Autopilot);
    const sim::FDMState state = simulation_->GetAircraft().ExtractFDMState(
        sim::FDMStateFlags::Controls);
    return math::RadToDeg(0.5
                          * (state.controls.leftAileronPositionRad
                              - state.controls.rightAileronPositionRad));
  }

  void Tick() {
    if (!simulation_->Tick()) {
      throw std::runtime_error("Simulation tick failed");
    }
  }

  Sample Capture() {
    const auto &aircraft = simulation_->GetAircraft();
    const auto &diagnostics = Autopilot(*simulation_).GetRollHoldDiagnostics();
    const sim::FDMState state = aircraft.ExtractFDMState(
        sim::FDMStateFlags::State | sim::FDMStateFlags::Controls);
    const double phi = state.state.attitudeRad[0];
    const double theta = state.state.attitudeRad[1];
    const double p = state.state.bodyAngularRatesRadPerSec[0];
    const double q = state.state.bodyAngularRatesRadPerSec[1];
    const double r = state.state.bodyAngularRatesRadPerSec[2];
    const double phiDot = p + q * std::sin(phi) * std::tan(theta)
                          + r * std::cos(phi) * std::tan(theta);
    return {
        simulation_->GetTime(),
        math::RadToDeg(diagnostics.bodyRateSetpointRadPerSec),
        math::RadToDeg(p),
        math::RadToDeg(phi),
        math::RadToDeg(q),
        math::RadToDeg(r),
        aircraft.GetProperties().Beta().Deg(),
        math::RadToDeg(diagnostics.bodyRateErrorRadPerSec),
        diagnostics.rateProportionalTerm,
        diagnostics.rateIntegralTerm,
        diagnostics.rateDerivativeTerm,
        diagnostics.rateFeedForwardTerm,
        diagnostics.rateIntegrator,
        diagnostics.unscaledTorqueCommand,
        diagnostics.rawTorqueCommand,
        diagnostics.rollTorqueCommand,
        diagnostics.aileronCommand,
        math::RadToDeg(0.5
                       * (state.controls.leftAileronPositionRad
                           - state.controls.rightAileronPositionRad)),
        math::RadToDeg(phiDot),
        diagnostics.airspeedScaling,
        diagnostics.positiveSaturation || diagnostics.negativeSaturation,
        diagnostics.integratorLimited,
    };
  }

  double hz_;
  double timeConstantSec_;
  std::unique_ptr<sim::Simulation> simulation_;
};

void MetricHeader(std::ostream &output) {
  output << "stage,simulation_hz,ff,p,i,d,mean_pulse_p_deg_s,"
            "steady_error_deg_s,rms_error_deg_s,max_error_deg_s,"
            "positive_peak_deg_s,overshoot_deg_s,first_undershoot_deg_s,"
            "second_rebound_deg_s,decay_ratio,settling_after_pulse_s,"
            "post_rms_p_deg_s,max_command,max_command_delta,max_surface_deg,"
            "max_surface_delta_deg,max_surface_slew_deg_s,"
            "command_variation_per_s,saturated,peak_r_deg_s,peak_beta_deg,"
            "post_rms_r_deg_s,post_rms_beta_deg,score,acceptable\n";
}

void MetricRow(std::ostream &output, const RateRun &run) {
  const Metrics &m = run.metrics;
  output << run.stage << ',' << run.hz << ',' << run.gains.ff << ','
         << run.gains.p << ',' << run.gains.i << ',' << run.gains.d << ','
         << m.meanPulseP << ',' << m.steadyError << ',' << m.rmsError << ','
         << m.maxError << ',' << m.positivePeak << ',' << m.overshoot << ','
         << m.firstUndershoot << ',' << m.secondRebound << ',' << m.decayRatio
         << ',' << FiniteOr(m.settling, -1.0) << ',' << m.postRmsP << ','
         << m.maxCommand << ',' << m.maxCommandDelta << ',' << m.maxSurface
         << ',' << m.maxSurfaceDelta << ',' << m.maxSurfaceSlew << ','
         << m.commandVariation << ',' << (m.saturated ? 1 : 0) << ',' << m.peakR
         << ',' << m.peakBeta << ',' << m.postRmsR << ',' << m.postRmsBeta
         << ',' << m.score << ',' << (Acceptable(m) ? 1 : 0) << '\n';
}

void WriteMetrics(const std::filesystem::path &path,
    const std::vector<RateRun> &runs) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Cannot write " + path.string());
  }
  output << std::setprecision(10);
  MetricHeader(output);
  for (const RateRun &run : runs) {
    MetricRow(output, run);
  }
}

void WriteSamples(const std::filesystem::path &path,
    const std::vector<RateRun> &runs) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Cannot write " + path.string());
  }
  output << std::setprecision(10)
         << "candidate,stage,simulation_hz,ff,p_gain,i,d,time_s,p_sp_deg_s,"
            "p_deg_s,phi_deg,q_deg_s,r_deg_s,beta_deg,rate_error_deg_s,"
            "p_term,i_term,d_term,ff_term,integrator,unscaled_torque,"
            "raw_torque,saturated_torque,aileron_command,surface_deg,"
            "reconstructed_phi_dot_deg_s,airspeed_scaling,saturated,"
            "integrator_limited\n";
  for (std::size_t index = 0; index < runs.size(); ++index) {
    const RateRun &run = runs[index];
    for (const Sample &s : run.samples) {
      output << index + 1 << ',' << run.stage << ',' << run.hz << ','
             << run.gains.ff << ',' << run.gains.p << ',' << run.gains.i << ','
             << run.gains.d << ',' << s.time << ',' << s.pSp << ',' << s.p
             << ',' << s.phi << ',' << s.q << ',' << s.r << ',' << s.beta << ','
             << s.rateError << ',' << s.pTerm << ',' << s.iTerm << ','
             << s.dTerm << ',' << s.ffTerm << ',' << s.integrator << ','
             << s.unscaledTorque << ',' << s.rawTorque << ','
             << s.saturatedTorque << ',' << s.aileron << ',' << s.surface << ','
             << s.reconstructedPhiDot << ',' << s.airspeedScaling << ','
             << (s.saturated ? 1 : 0) << ',' << (s.integratorLimited ? 1 : 0)
             << '\n';
    }
  }
}

std::vector<RateRun> TopThree(std::vector<RateRun> runs) {
  std::stable_sort(runs.begin(),
      runs.end(),
      [](const RateRun &left, const RateRun &right) {
        if (Acceptable(left.metrics) != Acceptable(right.metrics)) {
          return Acceptable(left.metrics);
        }
        return left.metrics.score < right.metrics.score;
      });
  if (runs.size() > 3) {
    runs.resize(3);
  }
  return runs;
}

std::vector<RateRun> CoarseSweep(Harness &harness) {
  constexpr std::array ffValues{0.8, 1.0, 1.2, 1.4, 1.6};
  constexpr std::array pValues{1.6, 2.0, 2.4, 2.8, 3.2, 3.6, 4.0};
  std::vector<RateRun> runs;
  for (double ff : ffValues) {
    for (double p : pValues) {
      runs.push_back(harness.DirectPulse("coarse", {ff, p, 0.0, 0.0}));
    }
  }
  return runs;
}

std::vector<RateRun> FineSweep(Harness &harness, const RateRun &best) {
  const double ffStart = std::max(0.05, best.gains.ff - 0.20);
  const double pStart = std::max(0.05, best.gains.p - 0.40);
  std::vector<RateRun> runs;
  for (int ffIndex = 0; ffIndex <= 8; ++ffIndex) {
    const double ff = std::round((ffStart + ffIndex * 0.05) * 100.0) / 100.0;
    for (int pIndex = 0; pIndex <= 8; ++pIndex) {
      const double p = std::round((pStart + pIndex * 0.10) * 100.0) / 100.0;
      runs.push_back(harness.DirectPulse("fine", {ff, p, 0.0, 0.0}));
    }
  }
  return runs;
}

double RobustScore(const RateRun &run30, const RateRun &run120) {
  const Metrics &a = run30.metrics;
  const Metrics &b = run120.metrics;
  const double mismatch =
      std::abs(a.steadyError - b.steadyError) / 0.50
      + std::abs(a.firstUndershoot - b.firstUndershoot) / 0.75
      + std::abs(FiniteOr(a.settling, 8.0) - FiniteOr(b.settling, 8.0)) / 3.0
      + std::abs(a.maxCommandDelta - b.maxCommandDelta) / 0.40;
  return 0.5 * (a.score + b.score) + 0.05 * mismatch;
}

std::vector<IRun> IntegralSweep(Harness &harness, Gains base) {
  constexpr std::array
      values{0.0, 0.02, 0.05, 0.10, 0.20, 0.25, 0.30, 0.35, 0.40};
  std::vector<IRun> runs;
  for (double value : values) {
    Gains gains = base;
    gains.i = value;
    runs.push_back(
        {value, harness.DirectPulse("i_pulse", gains), harness.Bias(gains)});
  }
  const double baselineBias = std::abs(runs.front().bias.steadyError);
  const Metrics &baselinePulse = runs.front().pulse.metrics;
  for (IRun &run : runs) {
    run.score =
        (std::abs(run.bias.steadyError) / std::max(0.10, baselineBias)
            + run.bias.tailRmsP / std::max(0.10, baselineBias)
            + run.pulse.metrics.score / std::max(0.01, baselinePulse.score))
        / 3.0;
    run.score += run.bias.saturated ? 2.0 : 0.0;
    run.acceptable =
        run.gain > 0.0
        && std::abs(run.bias.steadyError) <= std::max(0.10, baselineBias * 0.20)
        && run.bias.limitedFraction < 0.05
        && run.pulse.metrics.overshoot <= baselinePulse.overshoot * 1.15 + 0.10
        && FiniteOr(run.pulse.metrics.settling, 9.0)
               <= FiniteOr(baselinePulse.settling, 8.0) + 1.0;
  }
  return runs;
}

const IRun &ChooseIntegral(const std::vector<IRun> &runs) {
  const auto acceptable = std::find_if(runs.begin(),
      runs.end(),
      [](const IRun &run) { return run.acceptable; });
  if (acceptable != runs.end()) {
    return *acceptable;
  }
  return *std::min_element(runs.begin(),
      runs.end(),
      [](const IRun &left, const IRun &right) {
        return left.score < right.score;
      });
}

std::vector<DRun> DerivativeSweep(Harness &harness, Gains base) {
  constexpr std::array values{0.0, 0.005, 0.01, 0.02, 0.04};
  std::vector<DRun> runs;
  for (double value : values) {
    Gains gains = base;
    gains.d = value;
    runs.push_back({value, harness.DirectPulse("d_pulse", gains)});
  }
  const Metrics &baseline = runs.front().pulse.metrics;
  for (DRun &run : runs) {
    const Metrics &candidate = run.pulse.metrics;
    run.clearBenefit =
        run.gain > 0.0 && candidate.score <= baseline.score * 0.95
        && candidate.commandVariation <= baseline.commandVariation * 1.10 + 1e-6
        && candidate.maxSurfaceSlew <= baseline.maxSurfaceSlew * 1.10 + 1e-6
        && !candidate.saturated;
  }
  return runs;
}

const DRun &ChooseDerivative(const std::vector<DRun> &runs) {
  const auto best = std::min_element(runs.begin(),
      runs.end(),
      [](const DRun &left, const DRun &right) {
        if (left.clearBenefit != right.clearBenefit) {
          return left.clearBenefit;
        }
        return left.pulse.metrics.score < right.pulse.metrics.score;
      });
  return best->clearBenefit ? *best : runs.front();
}

void WriteISweep(const std::filesystem::path &path,
    const std::vector<IRun> &runs) {
  std::ofstream output(path);
  output << std::setprecision(10)
         << "i,pulse_score,pulse_steady_error_deg_s,pulse_overshoot_deg_s,"
            "pulse_undershoot_deg_s,pulse_settling_s,post_rms_p_deg_s,"
            "bias_mean_p_deg_s,bias_steady_error_deg_s,bias_tail_rms_p_deg_s,"
            "final_integrator,max_integrator,limited_fraction,"
            "mean_aileron_bias,saturated,combined_score,acceptable\n";
  for (const IRun &run : runs) {
    const Metrics &m = run.pulse.metrics;
    output << run.gain << ',' << m.score << ',' << m.steadyError << ','
           << m.overshoot << ',' << m.firstUndershoot << ','
           << FiniteOr(m.settling, -1.0) << ',' << m.postRmsP << ','
           << run.bias.meanTailP << ',' << run.bias.steadyError << ','
           << run.bias.tailRmsP << ',' << run.bias.finalIntegrator << ','
           << run.bias.maxIntegrator << ',' << run.bias.limitedFraction << ','
           << run.bias.meanAileronBias << ',' << (run.bias.saturated ? 1 : 0)
           << ',' << run.score << ',' << (run.acceptable ? 1 : 0) << '\n';
  }
}

void WriteDSweep(const std::filesystem::path &path,
    const std::vector<DRun> &runs) {
  std::ofstream output(path);
  output << std::setprecision(10);
  MetricHeader(output);
  for (const DRun &run : runs) {
    MetricRow(output, run.pulse);
  }
}

std::string HeatColor(double fraction) {
  fraction = std::clamp(fraction, 0.0, 1.0);
  const int red = static_cast<int>(45 + 195 * fraction);
  const int green = static_cast<int>(190 - 125 * fraction);
  return "rgb(" + std::to_string(red) + "," + std::to_string(green) + ",55)";
}

void WriteHeatmap(const std::filesystem::path &path,
    const std::vector<RateRun> &runs) {
  std::vector<double> ffValues;
  std::vector<double> pValues;
  for (const RateRun &run : runs) {
    ffValues.push_back(run.gains.ff);
    pValues.push_back(run.gains.p);
  }
  std::sort(ffValues.begin(), ffValues.end());
  ffValues.erase(std::unique(ffValues.begin(), ffValues.end()), ffValues.end());
  std::sort(pValues.begin(), pValues.end());
  pValues.erase(std::unique(pValues.begin(), pValues.end()), pValues.end());
  const auto scoreBounds = std::minmax_element(runs.begin(),
      runs.end(),
      [](const RateRun &left, const RateRun &right) {
        return left.metrics.score < right.metrics.score;
      });
  const double minScore = scoreBounds.first->metrics.score;
  const double maxScore = scoreBounds.second->metrics.score;
  const double width = 150.0 + 76.0 * pValues.size();
  const double height = 120.0 + 40.0 * ffValues.size();
  std::ofstream output(path);
  output
      << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
      << "\" height=\"" << height
      << "\"><rect width=\"100%\" height=\"100%\" fill=\"#111827\"/>"
      << "<g font-family=\"sans-serif\" fill=\"#e5e7eb\"><text x=\"18\" "
         "y=\"26\" font-size=\"18\">Fine FF/P score (lower is better)</text>";
  for (std::size_t pIndex = 0; pIndex < pValues.size(); ++pIndex) {
    output << "<text x=\"" << 116 + 76 * pIndex
           << "\" y=\"54\" text-anchor=\"middle\" font-size=\"11\">P="
           << Format(pValues[pIndex], 2) << "</text>";
  }
  for (std::size_t ffIndex = 0; ffIndex < ffValues.size(); ++ffIndex) {
    const double y = 65 + 40 * ffIndex;
    output << "<text x=\"72\" y=\"" << y + 24
           << "\" text-anchor=\"end\" font-size=\"11\">FF="
           << Format(ffValues[ffIndex], 2) << "</text>";
    for (std::size_t pIndex = 0; pIndex < pValues.size(); ++pIndex) {
      const auto run =
          std::find_if(runs.begin(), runs.end(), [&](const RateRun &candidate) {
            return candidate.gains.ff == ffValues[ffIndex]
                   && candidate.gains.p == pValues[pIndex];
          });
      const double fraction =
          (run->metrics.score - minScore) / std::max(1e-9, maxScore - minScore);
      const double x = 80 + 76 * pIndex;
      output << "<rect x=\"" << x << "\" y=\"" << y
             << "\" width=\"72\" height=\"36\" rx=\"3\" fill=\""
             << HeatColor(fraction) << "\"/><text x=\"" << x + 36 << "\" y=\""
             << y + 23
             << "\" text-anchor=\"middle\" font-size=\"11\" fill=\"white\">"
             << Format(run->metrics.score) << "</text>";
    }
  }
  output << "</g></svg>\n";
}

struct Bounds {
  double low;
  double high;
};

Bounds GetBounds(const std::vector<Sample> &samples, double Sample::*field,
    double include = 0.0) {
  Bounds bounds{include, include};
  for (const Sample &sample : samples) {
    bounds.low = std::min(bounds.low, sample.*field);
    bounds.high = std::max(bounds.high, sample.*field);
  }
  const double padding = std::max(0.05, (bounds.high - bounds.low) * 0.1);
  bounds.low -= padding;
  bounds.high += padding;
  return bounds;
}

std::string Points(const std::vector<Sample> &samples, double Sample::*field,
    Bounds bounds, double top, double height) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(2);
  for (const Sample &sample : samples) {
    const double x = 70.0 + 900.0 * sample.time / samples.back().time;
    const double y =
        top
        + height * (bounds.high - sample.*field) / (bounds.high - bounds.low);
    output << x << ',' << y << ' ';
  }
  return output.str();
}

void WriteTracePlot(const std::filesystem::path &path,
    const std::vector<RateRun> &runs, std::string_view title,
    double Sample::*first, double Sample::*second, std::string_view firstLabel,
    std::string_view secondLabel, std::string_view firstColor,
    std::string_view secondColor) {
  const double panelHeight = 210.0;
  const double totalHeight = 55.0 + panelHeight * runs.size();
  std::ofstream output(path);
  output << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1000\" height=\""
         << totalHeight
         << "\"><rect width=\"100%\" height=\"100%\" fill=\"#111827\"/>"
         << "<g font-family=\"sans-serif\" fill=\"#e5e7eb\"><text x=\"18\" "
            "y=\"26\" font-size=\"18\">"
         << title << "</text><text x=\"720\" y=\"26\" fill=\"" << firstColor
         << "\">" << firstLabel << "</text><text x=\"850\" y=\"26\" fill=\""
         << secondColor << "\">" << secondLabel << "</text>";
  for (std::size_t index = 0; index < runs.size(); ++index) {
    const RateRun &run = runs[index];
    const double top = 48.0 + panelHeight * index;
    Bounds bounds = GetBounds(run.samples, first);
    const Bounds other = GetBounds(run.samples, second);
    bounds.low = std::min(bounds.low, other.low);
    bounds.high = std::max(bounds.high, other.high);
    output << "<rect x=\"70\" y=\"" << top
           << "\" width=\"900\" height=\"165\" fill=\"#1f2937\" "
              "stroke=\"#4b5563\"/>"
           << "<text x=\"80\" y=\"" << top + 17 << "\" font-size=\"12\">#"
           << index + 1 << " FF=" << Format(run.gains.ff, 2)
           << " P=" << Format(run.gains.p, 2)
           << " score=" << Format(run.metrics.score)
           << "</text><polyline fill=\"none\" stroke=\"" << firstColor
           << "\" stroke-width=\"2\" points=\""
           << Points(run.samples, first, bounds, top, 165.0)
           << "\"/><polyline fill=\"none\" stroke=\"" << secondColor
           << "\" stroke-width=\"2\" points=\""
           << Points(run.samples, second, bounds, top, 165.0) << "\"/>";
  }
  output << "</g></svg>\n";
}

void WriteActuatorPlot(const std::filesystem::path &path,
    const std::vector<RateRun> &runs) {
  std::vector<RateRun> scaled = runs;
  for (RateRun &run : scaled) {
    for (Sample &sample : run.samples) {
      sample.surface /= 20.0;
    }
  }
  WriteTracePlot(path,
      scaled,
      "Aileron command vs effective surface (surface normalized by 20 deg)",
      &Sample::aileron,
      &Sample::surface,
      "command",
      "surface / 20",
      "#a78bfa",
      "#22d3ee");
}

void WriteFullRoll(const std::filesystem::path &directory,
    const FullRollRun &run) {
  std::ofstream metrics(directory / "full_roll_hold.csv");
  metrics << std::setprecision(10)
          << "simulation_hz,tc,ff,p,i,d,mean_final_roll_deg,steady_error_deg,"
             "overshoot_deg,"
             "settling_s,tail_range_deg,max_p_deg_s,max_command,peak_r_deg_s,"
             "peak_beta_deg,saturated\n"
          << run.hz << ',' << run.timeConstantSec << ',' << run.gains.ff << ','
          << run.gains.p << ',' << run.gains.i << ',' << run.gains.d << ','
          << run.meanFinalRoll << ',' << run.steadyError << ',' << run.overshoot
          << ',' << FiniteOr(run.settling, -1.0) << ',' << run.tailRange << ','
          << run.maxP << ',' << run.maxCommand << ',' << run.peakR << ','
          << run.peakBeta << ',' << (run.saturated ? 1 : 0) << '\n';
  WriteSamples(directory / "full_roll_hold_timeseries.csv",
      {RateRun{"full_roll_hold", run.hz, run.gains, {}, run.samples}});
}

void WriteReport(const std::filesystem::path &path, const RateRun &coarseBest,
    const std::vector<RateRun> &top, const std::vector<RateRun> &fine,
    const std::vector<std::pair<RateRun, RateRun>> &robust,
    const std::pair<RateRun, RateRun> &nominal, const std::vector<IRun> &iRuns,
    const IRun &selectedI, const std::vector<DRun> &dRuns,
    const DRun &selectedD, const FullRollRun &full) {
  double bestScore = std::numeric_limits<double>::infinity();
  for (const RateRun &run : fine) {
    if (Acceptable(run.metrics)) {
      bestScore = std::min(bestScore, run.metrics.score);
    }
  }
  if (!std::isfinite(bestScore)) {
    bestScore = std::min_element(fine.begin(),
        fine.end(),
        [](const RateRun &a, const RateRun &b) {
          return a.metrics.score < b.metrics.score;
        })->metrics.score;
  }
  double minFf = std::numeric_limits<double>::infinity();
  double maxFf = -std::numeric_limits<double>::infinity();
  double minP = std::numeric_limits<double>::infinity();
  double maxP = -std::numeric_limits<double>::infinity();
  for (const RateRun &run : fine) {
    if (Acceptable(run.metrics) && run.metrics.score <= bestScore * 1.15) {
      minFf = std::min(minFf, run.gains.ff);
      maxFf = std::max(maxFf, run.gains.ff);
      minP = std::min(minP, run.gains.p);
      maxP = std::max(maxP, run.gains.p);
    }
  }
  if (!std::isfinite(minFf)) {
    minFf = maxFf = nominal.second.gains.ff;
    minP = maxP = nominal.second.gains.p;
  }

  std::ofstream out(path);
  out << "# PX4 Roll Rate Automated Tuning Report\n\n"
      << "## Conditions and implementation\n\n"
      << "Every run creates a fresh `Simulation` and performs the same full "
         "trim, so kinematics, controls, propulsion, environment, controller "
         "state, and integrator state cannot leak between combinations. The "
         "harness then explicitly injects every FF/P/I/D value. Direct-rate "
         "tests command 0 deg/s through 5 s, +5 deg/s from 5-7 s, then 0 "
         "deg/s. Coarse/fine sweeps use the nominal 120 Hz rate; the top "
         "three are rerun at 120 and the retained 30 Hz robustness rate. "
         "Production defaults are not changed.\n\n"
      << "The current controller equation, in rad/s, is:\n\n"
      << "```text\n"
      << "p_sp = clamp(direct_command, -max_rate, +max_rate)\n"
      << "e = p_sp - body_p\n"
      << "u = P*e + I_state - D*p_dot + (FF/s)*p_sp\n"
      << "raw = u*s^2 + trim_aileron*s^2\n"
      << "aileron = clamp(raw, -1, +1),  s = trim_CAS/max(CAS, stall_CAS)\n"
      << "```\n\n"
      << "JSBSim actuator dynamics remain active. The effective surface "
         "telemetry is `(left_aileron-right_aileron)/2`; reconstructed Euler "
         "roll rate uses `p + q sin(phi) tan(theta) + r cos(phi) "
         "tan(theta)`.\n\n"
      << "## Search and score\n\n"
      << "Coarse FF = {0.8,1.0,1.2,1.4,1.6}; P = "
         "{1.6,2.0,2.4,2.8,3.2,3.6,4.0}. "
         "The best coarse point is FF="
      << Format(coarseBest.gains.ff, 2)
      << ", P=" << Format(coarseBest.gains.p, 2)
      << ". Fine search uses +/-0.20 FF at 0.05 and +/-0.40 P at 0.10 around "
         "that point.\n\n"
      << "The composite is the equal-weight mean of normalized steady error, "
         "overshoot, negative undershoot, settling, residual p RMS, actuator "
         "effort, r RMS, and beta RMS. Reference scales are respectively 0.50 "
         "deg/s, 1.00 deg/s, 0.75 deg/s, 3 s, 0.25 deg/s, 0.40 command/90 "
         "deg/s surface slew, 0.50 deg/s, and 0.50 deg. Components are capped "
         "and saturation is penalized; raw values remain in CSV.\n\n"
      << "Candidate guardrails are steady tracking error <=0.25 deg/s (5% of "
         "the command), overshoot <=1.75 deg/s, first undershoot magnitude "
         "<=1.25 deg/s, settling <=5 s, command delta <=0.50, and no "
         "saturation. The guardrail prevents a low-excitation but "
         "under-tracking point from winning solely through the "
         "lateral-coupling terms.\n\n"
      << "## Top three at nominal 120 Hz\n\n"
      << "|#|FF|P|Mean p|Error|Overshoot|Undershoot|Settling|Post RMS p|Cmd "
         "delta|RMS r|RMS beta|Score|\n"
      << "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n";
  for (std::size_t index = 0; index < top.size(); ++index) {
    const RateRun &run = top[index];
    const Metrics &m = run.metrics;
    out << '|' << index + 1 << '|' << Format(run.gains.ff, 2) << '|'
        << Format(run.gains.p, 2) << '|' << Format(m.meanPulseP) << '|'
        << Format(m.steadyError) << '|' << Format(m.overshoot) << '|'
        << Format(m.firstUndershoot) << '|' << Format(m.settling) << '|'
        << Format(m.postRmsP) << '|' << Format(m.maxCommandDelta) << '|'
        << Format(m.postRmsR) << '|' << Format(m.postRmsBeta) << '|'
        << Format(m.score) << "|\n";
  }
  out << "\n## 30/120 Hz robustness\n\n"
      << "|FF|P|Score 30|Score 120|Error 30/120|Undershoot 30/120|Settling "
         "30/120|Robust score|\n"
      << "|---:|---:|---:|---:|---:|---:|---:|---:|\n";
  for (const auto &[run30, run120] : robust) {
    out << '|' << Format(run30.gains.ff, 2) << '|' << Format(run30.gains.p, 2)
        << '|' << Format(run30.metrics.score) << '|'
        << Format(run120.metrics.score) << '|'
        << Format(run30.metrics.steadyError) << '/'
        << Format(run120.metrics.steadyError) << '|'
        << Format(run30.metrics.firstUndershoot) << '/'
        << Format(run120.metrics.firstUndershoot) << '|'
        << Format(run30.metrics.settling) << '/'
        << Format(run120.metrics.settling) << '|'
        << Format(RobustScore(run30, run120)) << "|\n";
  }
  out << "\n## Recommendation\n\n"
      << "- Recommended FF: **" << Format(nominal.second.gains.ff, 2) << "**\n"
      << "- Recommended P: **" << Format(nominal.second.gains.p, 2) << "**\n"
      << "- Acceptable FF range: **" << Format(minFf, 2) << "-"
      << Format(maxFf, 2) << "**\n"
      << "- Acceptable P range: **" << Format(minP, 2) << "-" << Format(maxP, 2)
      << "**\n"
      << "- Robustness scores (120 Hz nominal / 30 Hz): "
      << Format(nominal.second.metrics.score) << " / "
      << Format(nominal.first.metrics.score) << "\n\n"
      << "The nominal is the lowest robust score, not the largest gain. "
         "Residual p motion that co-occurs with r/beta is categorized as "
         "lateral/Dutch-roll excitation; motion that changes with P, timestep, "
         "command variation, or surface slew is controller/actuator-induced. "
         "The raw telemetry preserves both for independent inspection.\n\n"
      << "## I sweep\n\n"
      << "A held +" << Format(BiasCommand, 3)
      << " normalized aileron-bias disturbance is introduced at 5 s and "
         "evaluated over the final 5 s of a 35 s run.\n\n"
      << "|I|Pulse score|Bias error|Tail RMS p|Final integrator|Limited "
         "fraction|Acceptable|\n"
      << "|---:|---:|---:|---:|---:|---:|:---:|\n";
  for (const IRun &run : iRuns) {
    out << '|' << Format(run.gain, 3) << '|' << Format(run.pulse.metrics.score)
        << '|' << Format(run.bias.steadyError) << '|'
        << Format(run.bias.tailRmsP) << '|' << Format(run.bias.finalIntegrator)
        << '|' << Format(run.bias.limitedFraction) << '|'
        << (run.acceptable ? "yes" : "no") << "|\n";
  }
  out << "\nSelected I: **" << Format(selectedI.gain, 3)
      << "**. The rule chooses the smallest I removing at least 80% of P-only "
         "bias (0.10 deg/s floor), without persistent limiting or material "
         "transient degradation; otherwise it reports the lowest combined "
         "score.\n\n"
      << "## D sweep\n\n"
      << "|D|Score|Undershoot|Settling|Command variation/s|Surface slew|Clear "
         "benefit|\n"
      << "|---:|---:|---:|---:|---:|---:|:---:|\n";
  for (const DRun &run : dRuns) {
    const Metrics &m = run.pulse.metrics;
    out << '|' << Format(run.gain, 3) << '|' << Format(m.score) << '|'
        << Format(m.firstUndershoot) << '|' << Format(m.settling) << '|'
        << Format(m.commandVariation) << '|' << Format(m.maxSurfaceSlew) << '|'
        << (run.clearBenefit ? "yes" : "no") << "|\n";
  }
  out << "\nSelected D: **" << Format(selectedD.gain, 3)
      << "**. Non-zero D requires at least 5% score improvement with no more "
         "than 10% extra command variation or surface slew.\n\n"
      << "## Full Roll Hold validation\n\n"
      << "Direct mode is disabled for a +5 deg angle step. Steady error is "
      << Format(full.steadyError) << " deg, overshoot "
      << Format(full.overshoot) << " deg, 0.5 deg settling "
      << Format(full.settling) << " s, final-5-s roll range "
      << Format(full.tailRange) << " deg, peak |r|/|beta| "
      << Format(full.peakR) << " deg/s / " << Format(full.peakBeta)
      << " deg, saturation " << (full.saturated ? "yes" : "no") << ".\n\n"
      << "It is safe to proceed with the proposed I only when its row is "
         "acceptable and full Roll Hold is unsaturated without a growing "
         "long-period range. If direct-rate behavior is sound but this "
         "angle-step is not, adjust/analyze outer-loop TC rather than "
         "increasing rate P.\n\n"
      << "## Artifacts\n\n"
      << "- [All FF/P metrics](ff_p_sweep.csv)\n"
      << "- [Timestep robustness](timestep_robustness.csv)\n"
      << "- [I sweep](i_sweep.csv)\n"
      << "- [D sweep](d_sweep.csv)\n"
      << "- [Top telemetry](top3_timeseries.csv)\n"
      << "- [Score heatmap](score_heatmap.svg)\n"
      << "- [Rate tracking](top3_rate_tracking.svg)\n"
      << "- [r/beta](top3_lateral.svg)\n"
      << "- [Actuator](top3_actuator.svg)\n"
      << "- [Full Roll Hold](full_roll_hold.csv)\n";
}

struct Options {
  std::filesystem::path output = "px4-roll-tuning-results";
  bool fullRollOnly = false;
  bool biasOnly = false;
  double hz = 120.0;
  double timeConstantSec = 0.4;
  Gains gains{1.1, 3.2, 0.4, 0.0};
};

Options ParseOptions(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      std::cout << "Usage: px4_roll_tuning_probe [--output DIRECTORY] "
                   "[(--full-roll-only|--bias-only) [--hz VALUE] [--tc VALUE] "
                   "[--ff VALUE] "
                   "[--p VALUE] [--i VALUE] [--d VALUE]]\n";
      std::exit(0);
    }
    if (argument == "--full-roll-only") {
      options.fullRollOnly = true;
      continue;
    }
    if (argument == "--bias-only") {
      options.biasOnly = true;
      continue;
    }
    if (argument == "--output" && index + 1 < argc) {
      options.output = argv[++index];
      continue;
    }

    const auto readDouble = [&](double &destination) {
      if (index + 1 >= argc) {
        throw std::runtime_error("Missing value for " + argument);
      }
      std::size_t parsedCharacters = 0;
      const std::string value = argv[++index];
      destination = std::stod(value, &parsedCharacters);
      if (parsedCharacters != value.size() || !std::isfinite(destination)) {
        throw std::runtime_error(
            "Invalid value for " + argument + ": " + value);
      }
    };
    if (argument == "--hz") {
      readDouble(options.hz);
    } else if (argument == "--tc") {
      readDouble(options.timeConstantSec);
    } else if (argument == "--ff") {
      readDouble(options.gains.ff);
    } else if (argument == "--p") {
      readDouble(options.gains.p);
    } else if (argument == "--i") {
      readDouble(options.gains.i);
    } else if (argument == "--d") {
      readDouble(options.gains.d);
    } else {
      throw std::runtime_error("Unknown argument: " + argument);
    }
  }
  if (options.hz <= 0.0 || options.timeConstantSec <= 0.0) {
    throw std::runtime_error(
        "Simulation rate and time constant must be positive");
  }
  return options;
}
} // namespace

int main(int argc, char **argv) {
  try {
    const Options options = ParseOptions(argc, argv);
    const std::filesystem::path &output = options.output;
    std::filesystem::create_directories(output);
    if (options.fullRollOnly || options.biasOnly) {
      Harness harness(options.hz, options.timeConstantSec);
      if (options.biasOnly) {
        const BiasMetrics bias = harness.Bias(options.gains);
        std::ofstream metrics(output / "bias_hold.csv");
        metrics << std::setprecision(10)
                << "simulation_hz,tc,ff,p,i,d,steady_error_deg_s,"
                   "tail_rms_p_deg_s,final_integrator,max_integrator,"
                   "limited_fraction,mean_aileron_bias,saturated\n"
                << options.hz << ',' << options.timeConstantSec << ','
                << options.gains.ff << ',' << options.gains.p << ','
                << options.gains.i << ',' << options.gains.d << ','
                << bias.steadyError << ',' << bias.tailRmsP << ','
                << bias.finalIntegrator << ',' << bias.maxIntegrator << ','
                << bias.limitedFraction << ',' << bias.meanAileronBias << ','
                << (bias.saturated ? 1 : 0) << '\n';
        std::cout << std::fixed << std::setprecision(4)
                  << "Bias: tc=" << options.timeConstantSec
                  << " ff=" << options.gains.ff << " p=" << options.gains.p
                  << " i=" << options.gains.i << " d=" << options.gains.d
                  << " steady_error=" << bias.steadyError
                  << " tail_rms_p=" << bias.tailRmsP
                  << " final_integrator=" << bias.finalIntegrator
                  << " limited_fraction=" << bias.limitedFraction
                  << " saturation=" << (bias.saturated ? "yes" : "no") << '\n';
        return 0;
      }
      const FullRollRun full = harness.FullRoll(options.gains);
      WriteFullRoll(output, full);
      std::cout << std::fixed << std::setprecision(4)
                << "Full Roll Hold: tc=" << full.timeConstantSec
                << " ff=" << full.gains.ff << " p=" << full.gains.p
                << " i=" << full.gains.i << " d=" << full.gains.d
                << " steady_error=" << full.steadyError
                << " overshoot=" << full.overshoot
                << " settling=" << FiniteOr(full.settling, -1.0)
                << " tail_range=" << full.tailRange << " max_p=" << full.maxP
                << " max_command=" << full.maxCommand
                << " saturation=" << (full.saturated ? "yes" : "no") << '\n';
      return 0;
    }
    Harness at120Hz(120.0);
    std::cout << "[1/7] coarse FF/P sweep\n";
    const std::vector<RateRun> coarse = CoarseSweep(at120Hz);
    const RateRun coarseBest = TopThree(coarse).front();
    std::cout << "[2/7] adaptive fine FF/P sweep around FF="
              << Format(coarseBest.gains.ff, 2)
              << " P=" << Format(coarseBest.gains.p, 2) << '\n';
    const std::vector<RateRun> fine = FineSweep(at120Hz, coarseBest);
    const std::vector<RateRun> top = TopThree(fine);

    std::cout << "[3/7] top-three 30/120 Hz robustness\n";
    Harness at30Hz(30.0);
    std::vector<std::pair<RateRun, RateRun>> robust;
    std::vector<RateRun> robustRows;
    std::vector<RateRun> topTelemetry;
    for (const RateRun &candidate : top) {
      RateRun run30 = at30Hz.DirectPulse("robustness", candidate.gains, true);
      RateRun run120 = at120Hz.DirectPulse("robustness", candidate.gains);
      topTelemetry.push_back(run120);
      robustRows.push_back(run30);
      robustRows.push_back(run120);
      robust.emplace_back(std::move(run30), std::move(run120));
    }
    const auto nominalIt = std::min_element(robust.begin(),
        robust.end(),
        [](const auto &left, const auto &right) {
          return RobustScore(left.first, left.second)
                 < RobustScore(right.first, right.second);
        });
    const auto &nominal = *nominalIt;

    std::cout << "[4/7] I pulse and held-bias sweep\n";
    const std::vector<IRun> iRuns =
        IntegralSweep(at120Hz, nominal.second.gains);
    const IRun &selectedI = ChooseIntegral(iRuns);
    Gains selected = nominal.second.gains;
    selected.i = selectedI.gain;

    std::cout << "[5/7] D benefit sweep\n";
    const std::vector<DRun> dRuns = DerivativeSweep(at120Hz, selected);
    const DRun &selectedD = ChooseDerivative(dRuns);
    selected.d = selectedD.gain;

    std::cout << "[6/7] full Roll Hold angle-step validation\n";
    const FullRollRun full = at120Hz.FullRoll(selected);
    std::cout << "[7/7] writing artifacts\n";
    std::vector<RateRun> all = coarse;
    all.insert(all.end(), fine.begin(), fine.end());
    WriteMetrics(output / "ff_p_sweep.csv", all);
    WriteMetrics(output / "timestep_robustness.csv", robustRows);
    WriteISweep(output / "i_sweep.csv", iRuns);
    WriteDSweep(output / "d_sweep.csv", dRuns);
    WriteSamples(output / "top3_timeseries.csv", topTelemetry);
    WriteHeatmap(output / "score_heatmap.svg", fine);
    WriteTracePlot(output / "top3_rate_tracking.svg",
        topTelemetry,
        "Top candidates: p_sp vs body p",
        &Sample::pSp,
        &Sample::p,
        "p_sp",
        "body p",
        "#f59e0b",
        "#60a5fa");
    WriteTracePlot(output / "top3_lateral.svg",
        topTelemetry,
        "Top candidates: lateral coupling",
        &Sample::r,
        &Sample::beta,
        "r (deg/s)",
        "beta (deg)",
        "#f472b6",
        "#34d399");
    WriteActuatorPlot(output / "top3_actuator.svg", topTelemetry);
    WriteFullRoll(output, full);
    WriteReport(output / "report.md",
        coarseBest,
        top,
        fine,
        robust,
        nominal,
        iRuns,
        selectedI,
        dRuns,
        selectedD,
        full);

    std::cout << "Completed: FF=" << Format(selected.ff, 2)
              << " P=" << Format(selected.p, 2)
              << " I=" << Format(selected.i, 3)
              << " D=" << Format(selected.d, 3)
              << "\nReport: " << std::filesystem::absolute(output / "report.md")
              << '\n';
  } catch (const std::exception &error) {
    std::cerr << "px4_roll_tuning_probe: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
