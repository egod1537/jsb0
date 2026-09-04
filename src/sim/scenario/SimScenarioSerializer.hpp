#pragma once

#include "sim/scenario/SimScenario.hpp"
#include "sim/execution/ExecutionVariant.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sim {
struct ScenarioLoadMetadata {
  std::optional<ExecutionVariant> legacyVariant;
  std::string sourceFile;
  std::string sourceDigestSha256;
  std::vector<std::string> warnings;
};

class SimScenarioSerializer {
public:
  // YAML conversion
  static std::string Serialize(const SimScenario &scenario);
  static bool Deserialize(std::string_view yaml,
      SimScenario &scenario, std::string &error,
      ScenarioLoadMetadata *metadata = nullptr);

  // File persistence
  static bool Load(const std::filesystem::path &path,
      SimScenario &scenario, std::string &error,
      ScenarioLoadMetadata *metadata = nullptr);
  static bool Save(const std::filesystem::path &path,
      const SimScenario &scenario, std::string &error);
};
} // namespace sim
