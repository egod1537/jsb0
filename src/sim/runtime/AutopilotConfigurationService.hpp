#pragma once

#include <string>

namespace sim {
class Simulation;
struct BaselineRollHoldConfig;
struct PrimaryRollHoldConfig;
struct ResolvedExecutionSpec;

class AutopilotConfigurationService {
public:
  static bool ApplyPrimary(Simulation &simulation,
      const PrimaryRollHoldConfig &config, bool &tuningChanged);
  static bool ApplyBaseline(Simulation &simulation,
      const BaselineRollHoldConfig &config, bool &tuningChanged);
  static bool ApplyExecutionParameters(Simulation &simulation,
      const ResolvedExecutionSpec &execution, std::string &error);
};
} // namespace sim
