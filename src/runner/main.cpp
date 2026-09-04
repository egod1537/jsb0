#include "McapRunObserver.hpp"
#include "RunnerOptions.hpp"
#include "SimRunner.hpp"

#include <csignal>
#include <exception>
#include <iostream>
#include <string_view>
#include <vector>

namespace {
volatile std::sig_atomic_t running = 1;

void OnSignal(int) { running = 0; }
} // namespace

int main(int argc, char **argv) {
  std::vector<std::string_view> arguments;
  arguments.reserve(argc > 1 ? static_cast<std::size_t>(argc - 1) : 0);
  for (int index = 1; index < argc; ++index) {
    arguments.emplace_back(argv[index]);
  }

  const runner::RunnerParseResult parsed =
      runner::ParseRunnerOptions(arguments);
  if (parsed.helpRequested) {
    runner::PrintRunnerHelp();
    return 0;
  }
  if (!parsed.options) {
    std::cerr << "[runner] argument error: " << parsed.error << '\n';
    runner::PrintRunnerHelp();
    return static_cast<int>(runner::RunnerExitCode::InvalidArguments);
  }

  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  runner::McapRunObserver recorder;
  runner::SimRunner simRunner;
  simRunner.AddObserver(recorder);
  try {
    const runner::RunnerResult result =
        simRunner.Run(*parsed.options, &running);
    if (!result.error.empty()) {
      std::cerr << "[runner] error: " << result.error << '\n';
    }
    return static_cast<int>(result.exitCode);
  } catch (const std::exception &exception) {
    std::cerr << "[runner] unhandled failure: " << exception.what() << '\n';
  } catch (...) {
    std::cerr << "[runner] unhandled failure: unknown exception\n";
  }
  return static_cast<int>(runner::RunnerExitCode::GeneralFailure);
}
