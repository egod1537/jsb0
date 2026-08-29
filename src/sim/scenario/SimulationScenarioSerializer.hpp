#pragma once

#include "sim/scenario/SimulationScenario.hpp"

#include <filesystem>
#include <string>
#include <string_view>

namespace sim {
class SimulationScenarioSerializer {
public:
  // YAML conversion
  static std::string Serialize(const SimulationScenario &scenario);
  static bool Deserialize(std::string_view yaml,
      SimulationScenario &scenario, std::string &error);

  // File persistence
  static bool Load(const std::filesystem::path &path,
      SimulationScenario &scenario, std::string &error);
  static bool Save(const std::filesystem::path &path,
      const SimulationScenario &scenario, std::string &error);
};
} // namespace sim
