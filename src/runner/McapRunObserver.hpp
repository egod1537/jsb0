#pragma once

#include "SimRunner.hpp"
#include "sim/telemetry/recording/TelemetryRecordingService.hpp"

namespace runner {
class McapRunObserver final : public ISimRunObserver {
public:
  bool OnRunStarted(const SimRunInfo &info,
      const SimRunObservation &observation, std::string &error) override;
  bool OnSimStep(const SimRunInfo &info,
      const SimRunObservation &observation, std::string &error) override;
  bool OnRunFinished(const SimRunInfo &info, const RunnerResult &result,
      std::string &error) override;

private:
  bool Consume(const SimRunObservation &observation, std::string &error);

  telemetry::recording::TelemetryRecordingService recording_;
  bool started_ = false;
};
} // namespace runner
