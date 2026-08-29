#pragma once

#include "sim/gnc/autopilot/AutopilotFactory.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace runner {
struct RunnerOptions {
  std::filesystem::path scenarioPath;
  std::filesystem::path outputDirectory;
  std::optional<gnc::AutopilotKind> autopilot;
  std::optional<double> dtSec;
  std::optional<double> durationSec;
  bool noTrim = false;
};

struct RunnerParseResult {
  std::optional<RunnerOptions> options;
  bool helpRequested = false;
  std::string error;
};

RunnerParseResult ParseRunnerOptions(
    const std::vector<std::string_view> &arguments);
void PrintRunnerHelp();
} // namespace runner
