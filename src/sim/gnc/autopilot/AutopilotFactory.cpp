#include "sim/gnc/autopilot/AutopilotFactory.hpp"

#include "sim/gnc/autopilot/experimental/ExperimentalAutopilotFactory.hpp"
#include "sim/gnc/autopilot/IAutopilot.hpp"
#include "sim/gnc/autopilot/px4/Px4AutopilotFactory.hpp"

namespace gnc {
std::unique_ptr<IAutopilot> CreateAutopilot(AutopilotKind kind) {
  switch (kind) {
  case AutopilotKind::Primary:
    return CreateExperimentalAutopilot();
  case AutopilotKind::Baseline:
    return CreateC172xPx4Autopilot();
  }
  return nullptr;
}

const char *ToString(AutopilotKind kind) {
  switch (kind) {
  case AutopilotKind::Primary:
    return "primary";
  case AutopilotKind::Baseline:
    return "baseline";
  }
  return "unknown";
}

bool TryParseAutopilotKind(std::string_view value, AutopilotKind &kind) {
  if (value == "primary") {
    kind = AutopilotKind::Primary;
    return true;
  }
  if (value == "baseline") {
    kind = AutopilotKind::Baseline;
    return true;
  }
  return false;
}

std::optional<AutopilotKind> IdentifyAutopilotKind(
    const IAutopilot &autopilot) {
  if (IsExperimentalAutopilot(autopilot)) {
    return AutopilotKind::Primary;
  }
  if (IsPx4Autopilot(autopilot)) {
    return AutopilotKind::Baseline;
  }
  return std::nullopt;
}
} // namespace gnc
