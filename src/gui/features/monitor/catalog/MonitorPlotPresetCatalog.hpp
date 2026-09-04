#pragma once

#include "gui/features/monitor/MonitorModel.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace gui {
inline constexpr std::size_t MaximumMonitorPresetPlots = 16;

enum class DefaultTelemetryPlot : std::size_t {
  AerodynamicAngles,
  Attitude,
  BodyVelocities,
  BodyRates,
  Airspeed,
  AltitudeAgl,
  BodyAccelerations,
  AngularAccelerations,
  CourseHoldCourseTracking,
  CourseHoldCourseError,
  CourseHoldCourseToRoll,
  RollHoldRollTracking,
  RollHoldRollError,
  RollHoldRollRateTracking,
  RollHoldRollRateError,
  Px4RollHoldRateControllerTerms,
  RollHoldControlOutput,
  Px4RollHoldIntegrator,
  Px4RollHoldCalibratedAirspeed,
  Px4RollHoldAirspeedScaling,
  Px4RollHoldLateralRates,
  Px4RollHoldSideslip,
  Px4RollHoldLateralControls,
  Px4RollHoldSaturationStatus,
  Px4YawRateTracking,
  Px4YawSideslipFeedback,
  Px4YawControllerTerms,
  Px4YawRudderOutput,
  Px4YawSaturationStatus,
  PitchHoldPitchTracking,
  PitchHoldPitchError,
  PitchHoldPitchRateTracking,
  PitchHoldPitchRateError,
  Px4PitchHoldRateControllerTerms,
  PitchHoldControlOutput,
  Px4PitchHoldIntegrator,
  Px4PitchHoldCalibratedAirspeed,
  Px4PitchHoldAirspeedScaling,
  Px4PitchHoldAngleOfAttack,
  Px4PitchHoldPitchAcceleration,
  Px4PitchHoldElevator,
  Px4PitchHoldSaturationStatus,
  TecsAltitude,
  TecsAirspeed,
  TecsEnergy,
  TecsCommands,
  Count,
};

struct TelemetryPlotBinding {
  std::string_view nodePath;
  std::string_view plotTitle;
  std::string_view yAxisLabel;
};

enum class MonitorPreset : std::size_t {
  AircraftState,
  Px4CourseHold,
  RollHold,
  Px4RollHoldDiagnostics,
  Px4RollHoldITuning,
  Px4YawCoordination,
  Px4PitchHoldDiagnostics,
  Px4Tecs,
  Count,
};

enum class MonitorPresetCategory {
  Aircraft,
  Controllers,
};

struct MonitorPresetCategoryDefinition {
  MonitorPresetCategory category;
  std::string_view name;
};

struct MonitorPresetDefinition {
  MonitorPreset preset;
  MonitorPresetCategory category;
  std::string_view name;
  std::array<DefaultTelemetryPlot, MaximumMonitorPresetPlots> requiredPlots;
  std::size_t requiredPlotCount;
};

const TelemetryPlotBinding &GetTelemetryPlotBinding(DefaultTelemetryPlot plot);
std::span<const MonitorPresetCategoryDefinition>
GetMonitorPresetCategoryDefinitions();
std::span<const MonitorPresetDefinition> GetMonitorPresetDefinitions();
std::vector<MonitorPlotState> BuildDefaultMonitorPlotTemplates();

constexpr std::uint32_t GetPresetBit(MonitorPreset preset) {
  return std::uint32_t{1} << static_cast<std::size_t>(preset);
}

std::uint32_t GetAllMonitorPresetMask();
bool IsMonitorPresetActive(std::uint32_t activeMask, std::size_t presetIndex);
bool IsMonitorPlotVisibleByPreset(std::string_view telemetryGroupPath,
    std::uint32_t activeMask);
} // namespace gui
