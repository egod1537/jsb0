#include "RunnerOptions.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <utility>

namespace runner {
namespace {
bool ParsePositiveDouble(std::string_view text, double &value) {
  if (text.empty()) {
    return false;
  }
  std::string copy(text);
  char *end = nullptr;
  errno = 0;
  value = std::strtod(copy.c_str(), &end);
  return errno == 0 && end == copy.c_str() + copy.size() && std::isfinite(value)
         && value > 0.0;
}

bool TakeValue(const std::vector<std::string_view> &arguments,
    std::size_t &index, std::string_view option, std::string_view &value,
    std::string &error) {
  if (index + 1 >= arguments.size() || arguments[index + 1].empty()) {
    error = std::string(option) + " requires a value";
    return false;
  }
  value = arguments[++index];
  return true;
}
} // namespace

RunnerParseResult ParseRunnerOptions(
    const std::vector<std::string_view> &arguments) {
  RunnerParseResult result;
  RunnerOptions options;
  bool scenarioSet = false;
  bool outputSet = false;
  for (std::size_t index = 0; index < arguments.size(); ++index) {
    const std::string_view argument = arguments[index];
    if (argument == "--help" || argument == "-h") {
      result.helpRequested = true;
      continue;
    }
    std::string_view value;
    if (argument == "--scenario") {
      if (!TakeValue(arguments, index, argument, value, result.error)) {
        return result;
      }
      if (scenarioSet) {
        result.error = "--scenario may only be specified once";
        return result;
      }
      options.scenarioPath = value;
      scenarioSet = true;
    } else if (argument == "--output") {
      if (!TakeValue(arguments, index, argument, value, result.error)) {
        return result;
      }
      if (outputSet) {
        result.error = "--output may only be specified once";
        return result;
      }
      options.outputDirectory = value;
      outputSet = true;
    } else if (argument == "--autopilot") {
      if (!TakeValue(arguments, index, argument, value, result.error)) {
        return result;
      }
      gnc::AutopilotKind autopilot;
      if (!gnc::TryParseAutopilotKind(value, autopilot)) {
        result.error = "--autopilot must be primary or baseline";
        return result;
      }
      options.autopilot = autopilot;
    } else if (argument == "--dt" || argument == "--duration") {
      if (!TakeValue(arguments, index, argument, value, result.error)) {
        return result;
      }
      double parsed = 0.0;
      if (!ParsePositiveDouble(value, parsed)) {
        result.error = std::string(argument) + " must be a finite value > 0";
        return result;
      }
      if (argument == "--dt") {
        options.dtSec = parsed;
      } else {
        options.durationSec = parsed;
      }
    } else if (argument == "--no-trim") {
      options.noTrim = true;
    } else {
      result.error = "unknown option: " + std::string(argument);
      return result;
    }
  }

  if (result.helpRequested) {
    return result;
  }
  if (!scenarioSet) {
    result.error = "--scenario is required";
    return result;
  }
  if (!outputSet) {
    result.error = "--output is required";
    return result;
  }
  result.options = std::move(options);
  return result;
}

void PrintRunnerHelp() {
  std::cout
      << "Usage:\n"
         "  jsb-sim-runner --scenario <file> --output <directory> [options]\n\n"
         "Options:\n"
         "  --scenario <path>          Scenario YAML file\n"
         "  --output <path>            Output directory\n"
         "  --autopilot <primary|baseline>  Override scenario autopilot\n"
         "  --dt <seconds>             Fixed timestep override\n"
         "  --duration <seconds>       Duration override\n"
         "  --no-trim                  Skip scenario trim\n"
         "  --help                     Show this help\n";
}
} // namespace runner
