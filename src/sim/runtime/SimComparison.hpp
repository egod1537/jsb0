#pragma once

#include "contract/telemetry/RecordingTypes.hpp"
#include "sim/execution/ExecutionRequest.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sim {
class SimRuntime;

enum class ComparisonExecutionState {
  Running,
  Completed,
  Stopped,
  Failed,
};

struct ComparisonVariantResult {
  ComparisonExecutionState state = ComparisonExecutionState::Stopped;
  std::string error;
};

struct ComparisonObservation {
  telemetry::recording::TelemetryFrame telemetry;
  std::vector<telemetry::recording::ScenarioEvent> scenarioEvents;
};

class SimComparison {
public:
  using RuntimeFactory = std::function<std::unique_ptr<SimRuntime>(
      const ResolvedExecutionSpec &, std::string &)>;

  ~SimComparison();

  SimComparison(const SimComparison &) = delete;
  SimComparison &operator=(const SimComparison &) = delete;

  static std::unique_ptr<SimComparison> Create(
      const ComparisonExecutionRequest &request, std::string &error,
      RuntimeFactory runtimeFactory = {},
      std::optional<ExecutionVariant> *failedVariant = nullptr);
  static std::unique_ptr<SimComparison> Create(
      const SimScenario &scenario, const ScenarioSource &source,
      std::string &error, RuntimeFactory runtimeFactory = {},
      std::optional<ExecutionVariant> *failedVariant = nullptr);

  // Synchronized execution lifecycle
  bool Tick();
  void Stop();
  void Shutdown();

  // Comparison state and output
  bool IsRunning() const;
  bool IsFinished() const;
  ComparisonExecutionState GetState() const;
  double GetSimulationTimeSec() const;
  std::uint64_t GetStepCount() const;
  const std::string &GetLastError() const;
  const ComparisonVariantResult &GetVariantResult(
      ExecutionVariant variant) const;
  ComparisonObservation TakeObservation();

private:
  SimComparison(std::unique_ptr<SimRuntime> baseline,
      std::unique_ptr<SimRuntime> primary, double dtSec,
      double durationSec);

  bool CollectEvents();
  std::optional<ExecutionVariant> FindDivergedClockVariant() const;
  bool Fail(ExecutionVariant variant, std::string error);

  // Independent variant runtimes
  std::unique_ptr<SimRuntime> baselineRuntime_;
  std::unique_ptr<SimRuntime> primaryRuntime_;

  // Shared execution plan and clock
  double dtSec_ = 0.0;
  double durationSec_ = 0.0;
  std::uint64_t stepCount_ = 0;
  ComparisonExecutionState state_ = ComparisonExecutionState::Running;

  // Aggregated results
  ComparisonVariantResult baselineResult_{ComparisonExecutionState::Running};
  ComparisonVariantResult primaryResult_{ComparisonExecutionState::Running};
  ComparisonObservation observation_;
  std::string lastError_;
};
} // namespace sim
