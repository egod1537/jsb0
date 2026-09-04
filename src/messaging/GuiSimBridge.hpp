#pragma once

#include "messaging/MessageBus.hpp"

#include <cstdint>
#include <vector>

namespace sim {
class SimRuntime;
}

namespace app::messaging {
class GuiSimBridge final {
public:
  GuiSimBridge(MessageBus &bus, sim::SimRuntime &runtime);

  // Runtime output publication
  void PublishState();
  void PublishTelemetry();

private:
  std::string GetRuntimeError(std::string fallback) const;

  // Dependencies
  MessageBus &bus_;
  sim::SimRuntime &runtime_;

  // Command subscription lifetime
  std::vector<Subscription> subscriptions_;

  // Published telemetry versions
  std::uint64_t primaryTelemetryVersion_ = 0;
  std::uint64_t baselineTelemetryVersion_ = 0;
};
} // namespace app::messaging
