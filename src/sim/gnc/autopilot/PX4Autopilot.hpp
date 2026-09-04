#pragma once

#include "sim/gnc/autopilot/IAutopilot.hpp"
#include "sim/gnc/autopilot/IControllerInspectable.hpp"
#include "sim/gnc/autopilot/IRollHoldAutopilot.hpp"
#include "sim/gnc/guidance/FixedWingSetpoint.hpp"
#include "sim/gnc/control/attitude/Px4PitchController.hpp"
#include "sim/gnc/control/attitude/Px4RollController.hpp"
#include "sim/gnc/control/lateral/Px4CourseController.hpp"
#include "sim/gnc/control/yaw/Px4YawRateController.hpp"
#include "sim/gnc/tecs/Px4TecsController.hpp"

namespace gnc {
struct Px4ControlProfile;

struct Px4AutopilotDiagnostics {
  Px4CourseHoldDiagnostics course;
  Px4RollHoldReferenceDiagnostics roll;
  Px4PitchHoldDiagnostics pitch;
  Px4TecsDiagnostics tecs;
  Px4YawRateDiagnostics yaw;
};

class PX4Autopilot final : public IAutopilot,
                           public IControllerInspectable,
                           public IRollHoldAutopilot {
public:
  PX4Autopilot();
  explicit PX4Autopilot(const Px4ControlProfile &profile);

  // Lifecycle and control output
  void Reset() override;
  control::ControlInput Update(sim::Aircraft &aircraft, const sim::Tick &tick,
      const control::ControlInput &passthroughCommand) override;
  Px4AutopilotDiagnostics GetDiagnostics() const;

  // Baseline Roll Hold
  bool IsRollHoldEnabled() const override;
  void SetRollHoldEnabled(bool enabled) override;
  double GetTargetRollRad() const override;
  void SetTargetRollRad(double targetRollRad) override;

  // PX4 reference settings and diagnostics
  const Px4RollHoldReferenceSettings &GetRollHoldSettings() const;
  void SetRollHoldSettings(const Px4RollHoldReferenceSettings &settings);
  const Px4RollHoldReferenceDiagnostics &GetRollHoldDiagnostics() const;

  // PX4 Pitch Hold
  bool IsPitchHoldEnabled() const;
  void SetPitchHoldEnabled(bool enabled);
  double GetTargetPitchRad() const;
  void SetTargetPitchRad(double targetPitchRad);
  const Px4PitchHoldSettings &GetPitchHoldSettings() const;
  void SetPitchHoldSettings(const Px4PitchHoldSettings &settings);
  const Px4PitchHoldDiagnostics &GetPitchHoldDiagnostics() const;

  // PX4 total-energy longitudinal outer loop
  bool IsTecsEnabled() const;
  void SetTecsEnabled(bool enabled);
  // TECS targets use metres AGL and metres per second CAS.
  double GetTargetAltitudeM() const;
  void SetTargetAltitudeM(double targetAltitudeM);
  double GetTargetAirspeedMps() const;
  void SetTargetAirspeedMps(double targetAirspeedMps);
  const Px4TecsSettings &GetTecsSettings() const;
  void SetTecsSettings(const Px4TecsSettings &settings);
  const Px4TecsDiagnostics &GetTecsDiagnostics() const;

  // PX4 course/lateral outer loop
  bool IsCourseHoldEnabled() const;
  void SetCourseHoldEnabled(bool enabled);
  double GetTargetCourseRad() const;
  void SetTargetCourseRad(double targetCourseRad);
  const Px4CourseHoldSettings &GetCourseHoldSettings() const;
  void SetCourseHoldSettings(const Px4CourseHoldSettings &settings);
  const Px4CourseHoldDiagnostics &GetCourseHoldDiagnostics() const;

  // PX4 yaw-rate control
  bool IsYawRateControlEnabled() const;
  void SetYawRateControlEnabled(bool enabled);
  const Px4YawRateSettings &GetYawRateSettings() const;
  void SetYawRateSettings(const Px4YawRateSettings &settings);
  const Px4YawRateDiagnostics &GetYawRateDiagnostics() const;

  // Trim reference consumption
  void SynchronizeTrimReferences(
      const AircraftTrimReference &trimReference) override;

private:
  // Controller lookup
  Controller *FindController(const std::type_info &type) override;
  const Controller *FindController(const std::type_info &type) const override;

  // Flight-control axes
  Px4CourseController courseController_;
  Px4RollController rollController_;
  Px4PitchController pitchController_;
  Px4TecsController tecsController_;
  Px4YawRateController yawRateController_;

  // Longitudinal mode ownership
  bool tecsEnabled_ = false;
  bool requestedPitchHoldEnabled_ = false;

  // Guidance and direct-hold targets
  FixedWingSetpoint setpoint_;
};
} // namespace gnc
