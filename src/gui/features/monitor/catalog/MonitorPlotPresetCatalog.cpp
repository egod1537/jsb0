#include "gui/features/monitor/catalog/MonitorPlotPresetCatalog.hpp"

#include "sim/telemetry/AircraftTelemetry.hpp"
#include "sim/telemetry/AutopilotTelemetry.hpp"

#include <initializer_list>

namespace gui {
namespace {
constexpr std::array<TelemetryPlotBinding,
    static_cast<std::size_t>(DefaultTelemetryPlot::Count)>
    TelemetryPlotBindings{{
        {"aircraft/aero", "Aerodynamic Angles", "rad"},
        {"aircraft/attitude", "Attitude", "rad"},
        {"aircraft/body_velocity", "Body Velocities", "m/s"},
        {"aircraft/rates", "Body Rates", "rad/s"},
        {"aircraft/airdata", "Airspeed", "m/s"},
        {"aircraft/position", "Altitude AGL", "m"},
        {"aircraft/body_acceleration", "Body Accelerations", "m/s^2"},
        {"aircraft/angular_acceleration", "Angular Accelerations", "rad/s^2"},
        {"preset/px4_course_hold/course_tracking", "Course Tracking", "rad"},
        {"preset/px4_course_hold/course_error", "Course Error", "rad"},
        {"preset/px4_course_hold/course_to_roll", "Course to Roll", "rad"},
        {"preset/roll_hold/roll_tracking", "Roll Attitude Tracking", "rad"},
        {"preset/px4_roll_hold/roll_error", "Roll Error", "rad"},
        {"preset/roll_hold/roll_rate_tracking", "Roll Rate Tracking", "rad/s"},
        {"preset/px4_roll_hold/roll_rate_error", "Roll Rate Error", "rad/s"},
        {"preset/px4_roll_hold/rate_terms",
            "PX4 Rate Controller Terms",
            "normalized"},
        {"preset/roll_hold/control_output",
            "Controller Output Pipeline",
            "normalized"},
        {"preset/px4_roll_hold/integrator", "Integrator", "normalized"},
        {"preset/px4_roll_hold/calibrated_airspeed",
            "Calibrated Airspeed CAS",
            "m/s"},
        {"preset/px4_roll_hold/airspeed_scaling",
            "Airspeed Scaling",
            "dimensionless"},
        {"preset/px4_roll_hold/lateral_rates",
            "Lateral Coupling Rates",
            "rad/s"},
        {"preset/px4_roll_hold/sideslip", "Lateral Coupling Beta", "rad"},
        {"preset/px4_roll_hold/lateral_controls",
            "Lateral Controls",
            "normalized"},
        {"preset/px4_roll_hold/saturation_status",
            "Saturation Status",
            "0 / 1"},
        {"preset/px4_yaw/rate_tracking", "Yaw Rate Tracking", "rad/s"},
        {"preset/px4_yaw/sideslip_feedback",
            "Sideslip Feedback",
            "rad / rad/s"},
        {"preset/px4_yaw/rate_terms", "PX4 Yaw Controller Terms", "normalized"},
        {"preset/px4_yaw/rudder_output", "Yaw / Rudder Output", "normalized"},
        {"preset/px4_yaw/saturation_status", "Yaw Saturation Status", "0 / 1"},
        {"preset/px4_pitch_hold/pitch_tracking",
            "Pitch Attitude Tracking",
            "rad"},
        {"preset/px4_pitch_hold/pitch_error", "Pitch Error", "rad"},
        {"preset/px4_pitch_hold/pitch_rate_tracking",
            "Pitch Rate Tracking",
            "rad/s"},
        {"preset/px4_pitch_hold/pitch_rate_error", "Pitch Rate Error", "rad/s"},
        {"preset/px4_pitch_hold/rate_terms",
            "PX4 Pitch Rate Controller Terms",
            "normalized"},
        {"preset/px4_pitch_hold/control_output",
            "Pitch Control Output Pipeline",
            "normalized"},
        {"preset/px4_pitch_hold/integrator", "Pitch Integrator", "normalized"},
        {"preset/px4_pitch_hold/calibrated_airspeed",
            "Pitch Hold Calibrated Airspeed CAS",
            "m/s"},
        {"preset/px4_pitch_hold/airspeed_scaling",
            "Pitch Hold Airspeed Scaling",
            "dimensionless"},
        {"preset/px4_pitch_hold/angle_of_attack",
            "Pitch Hold Angle of Attack",
            "rad"},
        {"preset/px4_pitch_hold/pitch_acceleration",
            "Pitch Angular Acceleration",
            "rad/s^2"},
        {"preset/px4_pitch_hold/elevator", "Pitch Hold Elevator", "normalized"},
        {"preset/px4_pitch_hold/saturation_status",
            "Pitch Saturation Status",
            "0 / 1"},
        {"preset/px4_tecs/altitude", "TECS Altitude AGL", "m"},
        {"preset/px4_tecs/airspeed", "TECS Airspeed CAS", "m/s"},
        {"preset/px4_tecs/energy", "TECS Energy Errors", "m^2/s^2"},
        {"preset/px4_tecs/commands", "TECS Commands", "rad / normalized"},
    }};

constexpr std::array MonitorPresetCategoryDefinitions{
    MonitorPresetCategoryDefinition{MonitorPresetCategory::Aircraft,
        "Aircraft"},
    MonitorPresetCategoryDefinition{MonitorPresetCategory::Controllers,
        "Controllers"},
};

constexpr std::array<MonitorPresetDefinition,
    static_cast<std::size_t>(MonitorPreset::Count)>
    MonitorPresetDefinitions{{
        {MonitorPreset::AircraftState,
            MonitorPresetCategory::Aircraft,
            "Aircraft State",
            {DefaultTelemetryPlot::AerodynamicAngles,
                DefaultTelemetryPlot::Attitude,
                DefaultTelemetryPlot::BodyVelocities,
                DefaultTelemetryPlot::BodyRates,
                DefaultTelemetryPlot::Airspeed,
                DefaultTelemetryPlot::AltitudeAgl,
                DefaultTelemetryPlot::BodyAccelerations,
                DefaultTelemetryPlot::AngularAccelerations},
            8},
        {MonitorPreset::Px4CourseHold,
            MonitorPresetCategory::Controllers,
            "PX4 Course Hold",
            {DefaultTelemetryPlot::CourseHoldCourseTracking,
                DefaultTelemetryPlot::CourseHoldCourseError,
                DefaultTelemetryPlot::CourseHoldCourseToRoll,
                DefaultTelemetryPlot::RollHoldRollRateTracking,
                DefaultTelemetryPlot::Px4RollHoldLateralRates,
                DefaultTelemetryPlot::Px4RollHoldSideslip,
                DefaultTelemetryPlot::Px4RollHoldLateralControls},
            7},
        {MonitorPreset::RollHold,
            MonitorPresetCategory::Controllers,
            "Roll Hold",
            {DefaultTelemetryPlot::RollHoldRollTracking,
                DefaultTelemetryPlot::RollHoldRollRateTracking,
                DefaultTelemetryPlot::RollHoldControlOutput},
            3},
        {MonitorPreset::Px4RollHoldDiagnostics,
            MonitorPresetCategory::Controllers,
            "PX4 Roll Hold Diagnostics",
            {DefaultTelemetryPlot::RollHoldRollTracking,
                DefaultTelemetryPlot::RollHoldRollError,
                DefaultTelemetryPlot::RollHoldRollRateTracking,
                DefaultTelemetryPlot::RollHoldRollRateError,
                DefaultTelemetryPlot::Px4RollHoldRateControllerTerms,
                DefaultTelemetryPlot::RollHoldControlOutput,
                DefaultTelemetryPlot::Px4RollHoldIntegrator,
                DefaultTelemetryPlot::Px4RollHoldCalibratedAirspeed,
                DefaultTelemetryPlot::Px4RollHoldAirspeedScaling,
                DefaultTelemetryPlot::Px4RollHoldLateralRates,
                DefaultTelemetryPlot::Px4RollHoldSideslip,
                DefaultTelemetryPlot::Px4RollHoldLateralControls,
                DefaultTelemetryPlot::Px4RollHoldSaturationStatus},
            13},
        {MonitorPreset::Px4RollHoldITuning,
            MonitorPresetCategory::Controllers,
            "PX4 Roll Hold I Tuning",
            {DefaultTelemetryPlot::RollHoldRollTracking,
                DefaultTelemetryPlot::RollHoldRollRateTracking,
                DefaultTelemetryPlot::RollHoldRollRateError,
                DefaultTelemetryPlot::Px4RollHoldRateControllerTerms,
                DefaultTelemetryPlot::Px4RollHoldIntegrator,
                DefaultTelemetryPlot::Px4RollHoldSaturationStatus},
            6},
        {MonitorPreset::Px4YawCoordination,
            MonitorPresetCategory::Controllers,
            "PX4 Yaw Coordination",
            {DefaultTelemetryPlot::RollHoldRollRateTracking,
                DefaultTelemetryPlot::Px4YawRateTracking,
                DefaultTelemetryPlot::Px4YawSideslipFeedback,
                DefaultTelemetryPlot::Px4YawControllerTerms,
                DefaultTelemetryPlot::Px4YawRudderOutput,
                DefaultTelemetryPlot::Px4YawSaturationStatus},
            6},
        {MonitorPreset::Px4PitchHoldDiagnostics,
            MonitorPresetCategory::Controllers,
            "PX4 Pitch Hold Diagnostics",
            {DefaultTelemetryPlot::PitchHoldPitchTracking,
                DefaultTelemetryPlot::PitchHoldPitchError,
                DefaultTelemetryPlot::PitchHoldPitchRateTracking,
                DefaultTelemetryPlot::PitchHoldPitchRateError,
                DefaultTelemetryPlot::Px4PitchHoldRateControllerTerms,
                DefaultTelemetryPlot::PitchHoldControlOutput,
                DefaultTelemetryPlot::Px4PitchHoldIntegrator,
                DefaultTelemetryPlot::Px4PitchHoldCalibratedAirspeed,
                DefaultTelemetryPlot::Px4PitchHoldAirspeedScaling,
                DefaultTelemetryPlot::Px4PitchHoldAngleOfAttack,
                DefaultTelemetryPlot::Px4PitchHoldPitchAcceleration,
                DefaultTelemetryPlot::Px4PitchHoldElevator,
                DefaultTelemetryPlot::Px4PitchHoldSaturationStatus},
            13},
        {MonitorPreset::Px4Tecs,
            MonitorPresetCategory::Controllers,
            "PX4 TECS",
            {DefaultTelemetryPlot::TecsAltitude,
                DefaultTelemetryPlot::TecsAirspeed,
                DefaultTelemetryPlot::TecsEnergy,
                DefaultTelemetryPlot::TecsCommands},
            4},
    }};

constexpr bool AreAllTelemetryPlotsAssignedToPreset() {
  for (std::size_t plotIndex = 0;
      plotIndex < static_cast<std::size_t>(DefaultTelemetryPlot::Count);
      ++plotIndex) {
    const auto plot = static_cast<DefaultTelemetryPlot>(plotIndex);
    bool assigned = false;
    for (const MonitorPresetDefinition &preset : MonitorPresetDefinitions) {
      for (std::size_t requiredIndex = 0;
          requiredIndex < preset.requiredPlotCount;
          ++requiredIndex) {
        assigned = assigned || preset.requiredPlots[requiredIndex] == plot;
      }
    }
    if (!assigned) {
      return false;
    }
  }
  return true;
}

static_assert(AreAllTelemetryPlotsAssignedToPreset());
} // namespace

const TelemetryPlotBinding &GetTelemetryPlotBinding(DefaultTelemetryPlot plot) {
  return TelemetryPlotBindings[static_cast<std::size_t>(plot)];
}

std::span<const MonitorPresetCategoryDefinition>
GetMonitorPresetCategoryDefinitions() {
  return MonitorPresetCategoryDefinitions;
}

std::span<const MonitorPresetDefinition> GetMonitorPresetDefinitions() {
  return MonitorPresetDefinitions;
}

std::vector<MonitorPlotState> BuildDefaultMonitorPlotTemplates() {
  std::vector<MonitorPlotState> plots;
  plots.reserve(static_cast<std::size_t>(DefaultTelemetryPlot::Count));
  const auto addPlot = [&plots](DefaultTelemetryPlot preset,
                           std::initializer_list<std::string_view>
                               paths) {
    const TelemetryPlotBinding &binding = GetTelemetryPlotBinding(preset);
    MonitorPlotState plot{.title = std::string(binding.plotTitle),
        .telemetryGroupPath = std::string(binding.nodePath),
        .yAxisLabel = std::string(binding.yAxisLabel)};
    for (const std::string_view path : paths) {
      plot.channels.emplace_back(path);
    }
    plots.push_back(std::move(plot));
  };

  addPlot(DefaultTelemetryPlot::AerodynamicAngles,
      {telemetry::paths::AircraftAeroAlpha,
          telemetry::paths::AircraftAeroBeta});
  addPlot(DefaultTelemetryPlot::Attitude,
      {telemetry::paths::AircraftAttitudeRoll,
          telemetry::paths::AircraftAttitudePitch,
          telemetry::paths::AircraftAttitudeHeading});
  addPlot(DefaultTelemetryPlot::BodyVelocities,
      {telemetry::paths::AircraftBodyVelocityU,
          telemetry::paths::AircraftBodyVelocityV,
          telemetry::paths::AircraftBodyVelocityW});
  addPlot(DefaultTelemetryPlot::BodyRates,
      {telemetry::paths::AircraftRateP,
          telemetry::paths::AircraftRateQ,
          telemetry::paths::AircraftRateR});
  addPlot(DefaultTelemetryPlot::Airspeed,
      {telemetry::paths::AircraftCalibratedAirspeed,
          telemetry::paths::AircraftTrueAirspeed});
  addPlot(DefaultTelemetryPlot::AltitudeAgl,
      {telemetry::paths::AircraftAltitudeAgl});
  addPlot(DefaultTelemetryPlot::BodyAccelerations,
      {telemetry::paths::AircraftBodyAccelerationU,
          telemetry::paths::AircraftBodyAccelerationV,
          telemetry::paths::AircraftBodyAccelerationW});
  addPlot(DefaultTelemetryPlot::AngularAccelerations,
      {telemetry::paths::AircraftAngularAccelerationP,
          telemetry::paths::AircraftAngularAccelerationQ,
          telemetry::paths::AircraftAngularAccelerationR});
  addPlot(DefaultTelemetryPlot::RollHoldRollTracking,
      {telemetry::paths::AutopilotRollHoldCommandedRoll,
          telemetry::paths::AutopilotRollHoldRoll});
  addPlot(DefaultTelemetryPlot::CourseHoldCourseTracking,
      {telemetry::paths::AutopilotCourseHoldCommandedCourse,
          telemetry::paths::AutopilotCourseHoldCourse});
  addPlot(DefaultTelemetryPlot::CourseHoldCourseError,
      {telemetry::paths::AutopilotCourseHoldCourseError});
  addPlot(DefaultTelemetryPlot::CourseHoldCourseToRoll,
      {telemetry::paths::AutopilotCourseHoldRawRollSetpoint,
          telemetry::paths::AutopilotCourseHoldLimitedRollSetpoint,
          telemetry::paths::AutopilotRollHoldRoll});
  addPlot(DefaultTelemetryPlot::RollHoldRollError,
      {telemetry::paths::AutopilotRollHoldRollError});
  addPlot(DefaultTelemetryPlot::RollHoldRollRateTracking,
      {telemetry::paths::AutopilotRollHoldCommandedRollRate,
          telemetry::paths::AutopilotRollHoldRollRate});
  addPlot(DefaultTelemetryPlot::RollHoldRollRateError,
      {telemetry::paths::AutopilotRollHoldRollRateError});
  addPlot(DefaultTelemetryPlot::Px4RollHoldRateControllerTerms,
      {telemetry::paths::AutopilotRollHoldRateProportionalTerm,
          telemetry::paths::AutopilotRollHoldRateIntegralTerm,
          telemetry::paths::AutopilotRollHoldRateDerivativeTerm,
          telemetry::paths::AutopilotRollHoldRateFeedForwardTerm});
  addPlot(DefaultTelemetryPlot::RollHoldControlOutput,
      {telemetry::paths::AutopilotRollHoldUnscaledTorqueCommand,
          telemetry::paths::AutopilotRollHoldRawTorqueCommand,
          telemetry::paths::AutopilotRollHoldRollTorqueCommand,
          telemetry::paths::AutopilotRollHoldAileronCommand});
  addPlot(DefaultTelemetryPlot::Px4RollHoldIntegrator,
      {telemetry::paths::AutopilotRollHoldRateIntegralTerm,
          telemetry::paths::AutopilotRollHoldRateIntegratorPositiveLimit,
          telemetry::paths::AutopilotRollHoldRateIntegratorNegativeLimit});
  addPlot(DefaultTelemetryPlot::Px4RollHoldCalibratedAirspeed,
      {telemetry::paths::AircraftCalibratedAirspeed});
  addPlot(DefaultTelemetryPlot::Px4RollHoldAirspeedScaling,
      {telemetry::paths::AutopilotRollHoldAirspeedScaling});
  addPlot(DefaultTelemetryPlot::Px4RollHoldLateralRates,
      {telemetry::paths::AircraftRateP, telemetry::paths::AircraftRateR});
  addPlot(DefaultTelemetryPlot::Px4RollHoldSideslip,
      {telemetry::paths::AircraftAeroBeta});
  addPlot(DefaultTelemetryPlot::Px4RollHoldLateralControls,
      {telemetry::paths::AircraftControlAileron,
          telemetry::paths::AircraftControlRudder});
  addPlot(DefaultTelemetryPlot::Px4RollHoldSaturationStatus,
      {telemetry::paths::AutopilotRollHoldPositiveSaturation,
          telemetry::paths::AutopilotRollHoldNegativeSaturation,
          telemetry::paths::AutopilotRollHoldIntegratorLimited});
  addPlot(DefaultTelemetryPlot::PitchHoldPitchTracking,
      {telemetry::paths::AutopilotPitchHoldCommandedPitch,
          telemetry::paths::AutopilotPitchHoldPitch});
  addPlot(DefaultTelemetryPlot::PitchHoldPitchError,
      {telemetry::paths::AutopilotPitchHoldPitchError});
  addPlot(DefaultTelemetryPlot::PitchHoldPitchRateTracking,
      {telemetry::paths::AutopilotPitchHoldCommandedPitchRate,
          telemetry::paths::AutopilotPitchHoldPitchRate});
  addPlot(DefaultTelemetryPlot::PitchHoldPitchRateError,
      {telemetry::paths::AutopilotPitchHoldPitchRateError});
  addPlot(DefaultTelemetryPlot::Px4PitchHoldRateControllerTerms,
      {telemetry::paths::AutopilotPitchHoldRateProportionalTerm,
          telemetry::paths::AutopilotPitchHoldRateIntegralTerm,
          telemetry::paths::AutopilotPitchHoldRateDerivativeTerm,
          telemetry::paths::AutopilotPitchHoldRateFeedForwardTerm});
  addPlot(DefaultTelemetryPlot::PitchHoldControlOutput,
      {telemetry::paths::AutopilotPitchHoldUnscaledTorqueCommand,
          telemetry::paths::AutopilotPitchHoldRawTorqueCommand,
          telemetry::paths::AutopilotPitchHoldPitchTorqueCommand,
          telemetry::paths::AutopilotPitchHoldElevatorCommand});
  addPlot(DefaultTelemetryPlot::Px4PitchHoldIntegrator,
      {telemetry::paths::AutopilotPitchHoldRateIntegralTerm,
          telemetry::paths::AutopilotPitchHoldIntegratorPositiveLimit,
          telemetry::paths::AutopilotPitchHoldIntegratorNegativeLimit});
  addPlot(DefaultTelemetryPlot::Px4PitchHoldCalibratedAirspeed,
      {telemetry::paths::AircraftCalibratedAirspeed});
  addPlot(DefaultTelemetryPlot::Px4PitchHoldAirspeedScaling,
      {telemetry::paths::AutopilotPitchHoldAirspeedScaling});
  addPlot(DefaultTelemetryPlot::Px4PitchHoldAngleOfAttack,
      {telemetry::paths::AircraftAeroAlpha});
  addPlot(DefaultTelemetryPlot::Px4PitchHoldPitchAcceleration,
      {telemetry::paths::AircraftAngularAccelerationQ});
  addPlot(DefaultTelemetryPlot::Px4PitchHoldElevator,
      {telemetry::paths::AutopilotPitchHoldElevatorCommand,
          telemetry::paths::AutopilotPitchHoldTrimElevatorCommand,
          telemetry::paths::AircraftControlElevator});
  addPlot(DefaultTelemetryPlot::Px4PitchHoldSaturationStatus,
      {telemetry::paths::AutopilotPitchHoldPositiveSaturation,
          telemetry::paths::AutopilotPitchHoldNegativeSaturation,
          telemetry::paths::AutopilotPitchHoldIntegratorLimited});
  addPlot(DefaultTelemetryPlot::TecsAltitude,
      {telemetry::paths::AutopilotTecsTargetAltitude,
          telemetry::paths::AutopilotTecsInternalAltitudeSetpoint,
          telemetry::paths::AutopilotTecsAltitude});
  addPlot(DefaultTelemetryPlot::TecsAirspeed,
      {telemetry::paths::AutopilotTecsTargetAirspeed,
          telemetry::paths::AutopilotTecsAirspeed});
  addPlot(DefaultTelemetryPlot::TecsEnergy,
      {telemetry::paths::AutopilotTecsTotalEnergyError,
          telemetry::paths::AutopilotTecsEnergyBalanceError});
  addPlot(DefaultTelemetryPlot::TecsCommands,
      {telemetry::paths::AutopilotTecsTargetPitch,
          telemetry::paths::AutopilotTecsTargetThrottle});
  addPlot(DefaultTelemetryPlot::Px4YawRateTracking,
      {telemetry::paths::AutopilotYawRateCommandedYawRate,
          telemetry::paths::AutopilotYawRateCoordinatedYawRate,
          telemetry::paths::AutopilotYawRateYawRate,
          telemetry::paths::AutopilotYawRateFeedbackYawRate});
  addPlot(DefaultTelemetryPlot::Px4YawSideslipFeedback,
      {telemetry::paths::AutopilotYawRateSideslip,
          telemetry::paths::AutopilotYawRateSideslipCorrection});
  addPlot(DefaultTelemetryPlot::Px4YawControllerTerms,
      {telemetry::paths::AutopilotYawRateProportionalTerm,
          telemetry::paths::AutopilotYawRateIntegralTerm,
          telemetry::paths::AutopilotYawRateDerivativeTerm,
          telemetry::paths::AutopilotYawRateFeedForwardTerm,
          telemetry::paths::AutopilotYawRateRollToYawFeedForwardTerm});
  addPlot(DefaultTelemetryPlot::Px4YawRudderOutput,
      {telemetry::paths::AutopilotYawRateUnscaledTorqueCommand,
          telemetry::paths::AutopilotYawRateRawTorqueCommand,
          telemetry::paths::AutopilotYawRateYawTorqueCommand,
          telemetry::paths::AutopilotYawRateRawRudderCommand,
          telemetry::paths::AutopilotYawRateRudderCommand,
          telemetry::paths::AircraftControlRudder});
  addPlot(DefaultTelemetryPlot::Px4YawSaturationStatus,
      {telemetry::paths::AutopilotYawRatePositiveSaturation,
          telemetry::paths::AutopilotYawRateNegativeSaturation,
          telemetry::paths::AutopilotYawRateIntegratorLimited});
  return plots;
}

std::uint32_t GetAllMonitorPresetMask() {
  std::uint32_t mask = 0;
  for (const MonitorPresetDefinition &preset : MonitorPresetDefinitions) {
    mask |= GetPresetBit(preset.preset);
  }
  return mask;
}

bool IsMonitorPresetActive(std::uint32_t activeMask, std::size_t presetIndex) {
  if (presetIndex >= MonitorPresetDefinitions.size()) {
    return false;
  }
  return (activeMask
             & GetPresetBit(MonitorPresetDefinitions[presetIndex].preset))
         != 0;
}

bool IsMonitorPlotVisibleByPreset(std::string_view telemetryGroupPath,
    std::uint32_t activeMask) {
  for (std::size_t presetIndex = 0;
      presetIndex < MonitorPresetDefinitions.size();
      ++presetIndex) {
    if (!IsMonitorPresetActive(activeMask, presetIndex)) {
      continue;
    }
    const MonitorPresetDefinition &preset =
        MonitorPresetDefinitions[presetIndex];
    for (std::size_t plotIndex = 0; plotIndex < preset.requiredPlotCount;
        ++plotIndex) {
      if (telemetryGroupPath
          == GetTelemetryPlotBinding(preset.requiredPlots[plotIndex])
              .nodePath) {
        return true;
      }
    }
  }
  return false;
}
} // namespace gui
