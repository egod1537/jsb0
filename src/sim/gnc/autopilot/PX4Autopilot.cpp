#include "sim/gnc/autopilot/PX4Autopilot.hpp"

#include "sim/Aircraft.hpp"
#include "sim/Tick.hpp"
#include "sim/gnc/config/Px4ControlProfile.hpp"
#include "sim/gnc/trim/AircraftTrimReference.hpp"

#include <algorithm>
#include <cmath>

namespace gnc {
PX4Autopilot::PX4Autopilot() = default;

PX4Autopilot::PX4Autopilot(const Px4ControlProfile &profile) {
  courseController_.SetSettings(profile.course);
  rollController_.SetSettings(profile.roll);
  pitchController_.SetSettings(profile.pitch);
  yawRateController_.SetSettings(profile.yaw);
  tecsController_.SetSettings(profile.tecs);
}

void PX4Autopilot::Reset() {
  courseController_.Reset();
  rollController_.Reset();
  pitchController_.Reset();
  tecsController_.Reset();
  yawRateController_.Reset();
}

control::ControlInput PX4Autopilot::Update(sim::Aircraft &aircraft,
    const sim::Tick &tick, const control::ControlInput &passthroughCommand) {
  control::ControlInput input = passthroughCommand;
  double effectiveRollTargetRad = setpoint_.lateral.rollRad;
  if (const auto courseRollSetpoint = courseController_.OnTick(aircraft,
          tick,
          setpoint_.lateral.courseRad)) {
    effectiveRollTargetRad = *courseRollSetpoint;
  }
  if (const auto aileronCommand =
          rollController_.OnTick(aircraft, tick, effectiveRollTargetRad)) {
    input.aileron = *aileronCommand;
  }
  double effectivePitchTargetRad = setpoint_.longitudinal.pitchRad;
  if (tecsEnabled_) {
    const auto &properties = aircraft.GetProperties();
    const Px4TecsOutput tecsOutput = tecsController_.Update({
        .altitudeM = properties.AltitudeAgl().M(),
        .verticalSpeedMps = properties.VerticalSpeed().Mps(),
        .calibratedAirspeedMps = properties.CalibratedAirspeed().Mps(),
        .targetAltitudeM = setpoint_.longitudinal.altitudeAglM,
        .targetAirspeedMps = setpoint_.longitudinal.calibratedAirspeedMps,
        .currentPitchRad = properties.Pitch().Rad(),
        .currentThrottle = passthroughCommand.throttle,
        .gravityMps2 = properties.GravityMps2(),
        .dtSec = tick.dtSec,
    });
    if (tecsOutput.valid) {
      effectivePitchTargetRad = tecsOutput.targetPitchRad;
      input.throttle = tecsOutput.targetThrottle;
    }
  }
  if (const auto elevatorCommand = pitchController_.OnTick(aircraft,
          tick,
          effectivePitchTargetRad)) {
    input.elevator = *elevatorCommand;
  }
  if (const auto rudderCommand =
          yawRateController_.OnTick(aircraft, tick, input.aileron)) {
    input.rudder = *rudderCommand;
  }
  control::ClampControlInput(input);
  return input;
}

Px4AutopilotDiagnostics PX4Autopilot::GetDiagnostics() const {
  return Px4AutopilotDiagnostics{
      .course = courseController_.GetDiagnostics(),
      .roll = rollController_.GetDiagnostics(),
      .pitch = pitchController_.GetDiagnostics(),
      .tecs = tecsController_.GetDiagnostics(),
      .yaw = yawRateController_.GetDiagnostics(),
  };
}

bool PX4Autopilot::IsRollHoldEnabled() const {
  return rollController_.IsEnabled();
}

void PX4Autopilot::SetRollHoldEnabled(bool enabled) {
  rollController_.SetEnabled(enabled);
}

double PX4Autopilot::GetTargetRollRad() const {
  return setpoint_.lateral.rollRad;
}

void PX4Autopilot::SetTargetRollRad(double targetRollRad) {
  setpoint_.lateral.rollRad = targetRollRad;
}

const Px4RollHoldReferenceSettings &PX4Autopilot::GetRollHoldSettings() const {
  return rollController_.GetSettings();
}

void PX4Autopilot::SetRollHoldSettings(
    const Px4RollHoldReferenceSettings &settings) {
  rollController_.SetSettings(settings);
}

const Px4RollHoldReferenceDiagnostics &
PX4Autopilot::GetRollHoldDiagnostics() const {
  return rollController_.GetDiagnostics();
}

bool PX4Autopilot::IsPitchHoldEnabled() const {
  return pitchController_.IsEnabled();
}

void PX4Autopilot::SetPitchHoldEnabled(bool enabled) {
  requestedPitchHoldEnabled_ = enabled;
  pitchController_.SetEnabled(enabled || tecsEnabled_);
}

double PX4Autopilot::GetTargetPitchRad() const {
  return setpoint_.longitudinal.pitchRad;
}

void PX4Autopilot::SetTargetPitchRad(double targetPitchRad) {
  setpoint_.longitudinal.pitchRad = targetPitchRad;
}

const Px4PitchHoldSettings &PX4Autopilot::GetPitchHoldSettings() const {
  return pitchController_.GetSettings();
}

void PX4Autopilot::SetPitchHoldSettings(const Px4PitchHoldSettings &settings) {
  pitchController_.SetSettings(settings);
}

const Px4PitchHoldDiagnostics &PX4Autopilot::GetPitchHoldDiagnostics() const {
  return pitchController_.GetDiagnostics();
}

bool PX4Autopilot::IsTecsEnabled() const { return tecsEnabled_; }

void PX4Autopilot::SetTecsEnabled(bool enabled) {
  if (tecsEnabled_ == enabled) {
    return;
  }

  tecsEnabled_ = enabled;
  tecsController_.Reset();
  pitchController_.SetEnabled(enabled || requestedPitchHoldEnabled_);
}

double PX4Autopilot::GetTargetAltitudeM() const {
  return setpoint_.longitudinal.altitudeAglM;
}

void PX4Autopilot::SetTargetAltitudeM(double targetAltitudeM) {
  if (std::isfinite(targetAltitudeM)) {
    setpoint_.longitudinal.altitudeAglM = targetAltitudeM;
  }
}

double PX4Autopilot::GetTargetAirspeedMps() const {
  return setpoint_.longitudinal.calibratedAirspeedMps;
}

void PX4Autopilot::SetTargetAirspeedMps(double targetAirspeedMps) {
  if (std::isfinite(targetAirspeedMps) && targetAirspeedMps > 0.0) {
    setpoint_.longitudinal.calibratedAirspeedMps = targetAirspeedMps;
  }
}

const Px4TecsSettings &PX4Autopilot::GetTecsSettings() const {
  return tecsController_.GetSettings();
}

void PX4Autopilot::SetTecsSettings(const Px4TecsSettings &settings) {
  tecsController_.SetSettings(settings);
}

const Px4TecsDiagnostics &PX4Autopilot::GetTecsDiagnostics() const {
  return tecsController_.GetDiagnostics();
}

bool PX4Autopilot::IsCourseHoldEnabled() const {
  return courseController_.IsEnabled();
}

void PX4Autopilot::SetCourseHoldEnabled(bool enabled) {
  courseController_.SetEnabled(enabled);
  if (enabled) {
    rollController_.SetEnabled(true);
  }
}

double PX4Autopilot::GetTargetCourseRad() const {
  return setpoint_.lateral.courseRad;
}

void PX4Autopilot::SetTargetCourseRad(double targetCourseRad) {
  setpoint_.lateral.courseRad = targetCourseRad;
}

const Px4CourseHoldSettings &PX4Autopilot::GetCourseHoldSettings() const {
  return courseController_.GetSettings();
}

void PX4Autopilot::SetCourseHoldSettings(
    const Px4CourseHoldSettings &settings) {
  courseController_.SetSettings(settings);
}

const Px4CourseHoldDiagnostics &PX4Autopilot::GetCourseHoldDiagnostics() const {
  return courseController_.GetDiagnostics();
}

bool PX4Autopilot::IsYawRateControlEnabled() const {
  return yawRateController_.IsEnabled();
}

void PX4Autopilot::SetYawRateControlEnabled(bool enabled) {
  yawRateController_.SetEnabled(enabled);
}

const Px4YawRateSettings &PX4Autopilot::GetYawRateSettings() const {
  return yawRateController_.GetSettings();
}

void PX4Autopilot::SetYawRateSettings(const Px4YawRateSettings &settings) {
  yawRateController_.SetSettings(settings);
}

const Px4YawRateDiagnostics &PX4Autopilot::GetYawRateDiagnostics() const {
  return yawRateController_.GetDiagnostics();
}

void PX4Autopilot::SynchronizeTrimReferences(
    const AircraftTrimReference &trimReference) {
  Px4RollHoldReferenceSettings settings = rollController_.GetSettings();
  settings.trimAirspeedMps = trimReference.calibratedAirspeedMps;
  settings.trimRollCommand = trimReference.aileron;
  rollController_.SetSettings(settings);

  Px4PitchHoldSettings pitchSettings = pitchController_.GetSettings();
  pitchSettings.trimAirspeedMps = settings.trimAirspeedMps;
  pitchSettings.stallAirspeedMps = settings.stallAirspeedMps;
  pitchSettings.trimElevatorCommand = trimReference.elevator;
  pitchController_.SetSettings(pitchSettings);

  Px4TecsSettings tecsSettings = tecsController_.GetSettings();
  tecsSettings.trimThrottle = trimReference.throttle;
  tecsController_.SetSettings(tecsSettings);
  if (!tecsEnabled_) {
    setpoint_.longitudinal.altitudeAglM = trimReference.altitudeAglM;
    setpoint_.longitudinal.calibratedAirspeedMps = settings.trimAirspeedMps;
  }

  Px4YawRateSettings yawSettings = yawRateController_.GetSettings();
  yawSettings.trimAirspeedMps = settings.trimAirspeedMps;
  yawSettings.stallAirspeedMps = settings.stallAirspeedMps;
  yawSettings.trimRudderCommand = trimReference.rudder;
  yawRateController_.SetSettings(yawSettings);
}

Controller *PX4Autopilot::FindController(const std::type_info &type) {
  if (type == typeid(Px4CourseController)) {
    return &courseController_;
  }
  if (type == typeid(Px4RollController)) {
    return &rollController_;
  }
  if (type == typeid(Px4PitchController)) {
    return &pitchController_;
  }
  if (type == typeid(Px4TecsController)) {
    return &tecsController_;
  }
  return type == typeid(Px4YawRateController) ? &yawRateController_ : nullptr;
}

const Controller *PX4Autopilot::FindController(
    const std::type_info &type) const {
  if (type == typeid(Px4CourseController)) {
    return &courseController_;
  }
  if (type == typeid(Px4RollController)) {
    return &rollController_;
  }
  if (type == typeid(Px4PitchController)) {
    return &pitchController_;
  }
  if (type == typeid(Px4TecsController)) {
    return &tecsController_;
  }
  return type == typeid(Px4YawRateController) ? &yawRateController_ : nullptr;
}
} // namespace gnc
