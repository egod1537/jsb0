#pragma once

namespace control {
class FlightControlManager;
}

namespace sim {
class Aircraft;
struct Tick;
} // namespace sim

namespace telemetry {
class TelemetryRegistry;

class SimTelemetryPublisher {
public:
  static void Publish(const sim::Aircraft &aircraft,
      const control::FlightControlManager &flightControls,
      const sim::Tick &tick, TelemetryRegistry &registry);

private:
  static void PublishAutopilot(const sim::Aircraft &aircraft,
      const control::FlightControlManager &flightControls,
      const sim::Tick &tick, TelemetryRegistry &registry);
  static void PublishAircraft(const sim::Aircraft &aircraft,
      const sim::Tick &tick, TelemetryRegistry &registry);
};
} // namespace telemetry
