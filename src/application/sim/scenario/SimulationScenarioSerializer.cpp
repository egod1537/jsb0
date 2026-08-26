#include "application/sim/scenario/SimulationScenarioSerializer.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace sim {
namespace {
const char *TrimModeName(gnc::TrimMode mode) {
  switch (mode) {
  case gnc::TrimMode::Longitudinal:
    return "Longitudinal";
  case gnc::TrimMode::Full:
    return "Full";
  case gnc::TrimMode::Ground:
    return "Ground";
  }
  return "Unknown";
}

gnc::TrimMode ParseTrimMode(const std::string &value) {
  if (value == "Longitudinal") {
    return gnc::TrimMode::Longitudinal;
  }
  if (value == "Full") {
    return gnc::TrimMode::Full;
  }
  if (value == "Ground") {
    return gnc::TrimMode::Ground;
  }
  throw std::runtime_error("trim.mode must be Longitudinal, Full, or Ground");
}

YAML::Node RequireMap(const YAML::Node &parent, const char *key,
    const std::string &path) {
  const YAML::Node node = parent[key];
  if (!node) {
    throw std::runtime_error("missing required field: " + path);
  }
  if (!node.IsMap()) {
    throw std::runtime_error(path + " must be a mapping");
  }
  return node;
}

template <typename T>
T ReadRequired(const YAML::Node &parent, const char *key,
    const std::string &path) {
  const YAML::Node node = parent[key];
  if (!node) {
    throw std::runtime_error("missing required field: " + path);
  }
  if (!node.IsScalar()) {
    throw std::runtime_error(path + " must be a scalar value");
  }
  try {
    return node.as<T>();
  } catch (const YAML::Exception &exception) {
    throw std::runtime_error(path + " has an invalid type: " + exception.msg);
  }
}

void Validate(const SimulationScenario &scenario) {
  std::string error;
  if (!ValidateSimulationScenario(scenario, &error)) {
    throw std::runtime_error(error);
  }
}

SimulationScenario ParseScenario(const YAML::Node &root) {
  if (!root || !root.IsMap()) {
    throw std::runtime_error("scenario YAML root must be a mapping");
  }

  SimulationScenario scenario;
  scenario.name = ReadRequired<std::string>(root, "name", "name");

  const YAML::Node initial =
      RequireMap(root, "initial_condition", "initial_condition");
  scenario.altitudeFt = ReadRequired<double>(initial,
      "altitude_ft",
      "initial_condition.altitude_ft");
  scenario.airspeedKts = ReadRequired<double>(initial,
      "airspeed_kts",
      "initial_condition.airspeed_kts");
  scenario.initialRollDeg =
      ReadRequired<double>(initial, "roll_deg", "initial_condition.roll_deg");
  scenario.initialPitchDeg =
      ReadRequired<double>(initial, "pitch_deg", "initial_condition.pitch_deg");
  scenario.initialHeadingDeg = ReadRequired<double>(initial,
      "heading_deg",
      "initial_condition.heading_deg");

  const YAML::Node environment = RequireMap(root, "environment", "environment");
  scenario.windEnabled = ReadRequired<bool>(environment,
      "wind_enabled",
      "environment.wind_enabled");

  const YAML::Node trim = RequireMap(root, "trim", "trim");
  scenario.runTrim = ReadRequired<bool>(trim, "enabled", "trim.enabled");
  scenario.trimMode =
      ParseTrimMode(ReadRequired<std::string>(trim, "mode", "trim.mode"));

  const YAML::Node command = RequireMap(root, "command", "command");
  scenario.commandStartSec =
      ReadRequired<double>(command, "start_sec", "command.start_sec");
  scenario.commandedRollDeg =
      ReadRequired<double>(command, "roll_deg", "command.roll_deg");

  const YAML::Node simulation = RequireMap(root, "simulation", "simulation");
  scenario.durationSec = ReadRequired<double>(simulation,
      "duration_sec",
      "simulation.duration_sec");

  const YAML::Node acceptance = RequireMap(root, "acceptance", "acceptance");
  scenario.settlingBandDeg = ReadRequired<double>(acceptance,
      "settling_band_deg",
      "acceptance.settling_band_deg");
  scenario.settlingTimeLimitSec = ReadRequired<double>(acceptance,
      "settling_time_limit_sec",
      "acceptance.settling_time_limit_sec");
  scenario.overshootLimitDeg = ReadRequired<double>(acceptance,
      "overshoot_limit_deg",
      "acceptance.overshoot_limit_deg");
  scenario.maxOscillationCycles = ReadRequired<double>(acceptance,
      "max_oscillation_cycles",
      "acceptance.max_oscillation_cycles");

  Validate(scenario);
  return scenario;
}
} // namespace

std::string SimulationScenarioSerializer::Serialize(
    const SimulationScenario &scenario) {
  Validate(scenario);

  YAML::Emitter output;
  output << YAML::BeginMap;
  output << YAML::Key << "name" << YAML::Value << scenario.name;

  output << YAML::Key << "initial_condition" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "altitude_ft" << YAML::Value << scenario.altitudeFt;
  output << YAML::Key << "airspeed_kts" << YAML::Value << scenario.airspeedKts;
  output << YAML::Key << "roll_deg" << YAML::Value << scenario.initialRollDeg;
  output << YAML::Key << "pitch_deg" << YAML::Value << scenario.initialPitchDeg;
  output << YAML::Key << "heading_deg" << YAML::Value
         << scenario.initialHeadingDeg;
  output << YAML::EndMap;

  output << YAML::Key << "environment" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "wind_enabled" << YAML::Value << scenario.windEnabled;
  output << YAML::EndMap;

  output << YAML::Key << "trim" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "enabled" << YAML::Value << scenario.runTrim;
  output << YAML::Key << "mode" << YAML::Value
         << TrimModeName(scenario.trimMode);
  output << YAML::EndMap;

  output << YAML::Key << "command" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "start_sec" << YAML::Value << scenario.commandStartSec;
  output << YAML::Key << "roll_deg" << YAML::Value << scenario.commandedRollDeg;
  output << YAML::EndMap;

  output << YAML::Key << "simulation" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "duration_sec" << YAML::Value << scenario.durationSec;
  output << YAML::EndMap;

  output << YAML::Key << "acceptance" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "settling_band_deg" << YAML::Value
         << scenario.settlingBandDeg;
  output << YAML::Key << "settling_time_limit_sec" << YAML::Value
         << scenario.settlingTimeLimitSec;
  output << YAML::Key << "overshoot_limit_deg" << YAML::Value
         << scenario.overshootLimitDeg;
  output << YAML::Key << "max_oscillation_cycles" << YAML::Value
         << scenario.maxOscillationCycles;
  output << YAML::EndMap;
  output << YAML::EndMap;

  if (!output.good()) {
    throw std::runtime_error(output.GetLastError());
  }
  return std::string(output.c_str()) + '\n';
}

bool SimulationScenarioSerializer::Deserialize(std::string_view yaml,
    SimulationScenario &scenario, std::string &error) {
  try {
    const YAML::Node root = YAML::Load(std::string(yaml));
    SimulationScenario parsed = ParseScenario(root);
    scenario = std::move(parsed);
    error.clear();
    return true;
  } catch (const YAML::Exception &exception) {
    error = "YAML parse error: " + exception.msg;
  } catch (const std::exception &exception) {
    error = "Scenario validation error: " + std::string(exception.what());
  }
  return false;
}

bool SimulationScenarioSerializer::Load(const std::filesystem::path &path,
    SimulationScenario &scenario, std::string &error) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    error = "Could not open scenario file: " + path.string();
    return false;
  }

  std::ostringstream buffer;
  buffer << input.rdbuf();
  if (input.bad()) {
    error = "Could not read scenario file: " + path.string();
    return false;
  }
  return Deserialize(buffer.str(), scenario, error);
}

bool SimulationScenarioSerializer::Save(const std::filesystem::path &path,
    const SimulationScenario &scenario, std::string &error) {
  try {
    const std::filesystem::path parent = path.parent_path();
    if (!parent.empty()) {
      std::error_code filesystemError;
      std::filesystem::create_directories(parent, filesystemError);
      if (filesystemError) {
        error =
            "Could not create scenario directory: " + filesystemError.message();
        return false;
      }
    }

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) {
      error = "Could not open scenario file for writing: " + path.string();
      return false;
    }
    output << Serialize(scenario);
    if (!output) {
      error = "Could not write scenario file: " + path.string();
      return false;
    }
    error.clear();
    return true;
  } catch (const std::exception &exception) {
    error = "Could not save scenario: " + std::string(exception.what());
    return false;
  }
}
} // namespace sim
