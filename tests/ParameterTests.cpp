#include "sim/gnc/config/Px4ControlProfile.hpp"
#include "sim/gnc/parameters/Parameter.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {
constexpr double Tolerance = 1.0e-9;

void Require(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

void RequireNear(double actual, double expected, const std::string &message) {
  Require(std::abs(actual - expected) <= Tolerance,
      message + ": expected " + std::to_string(expected) + ", got "
          + std::to_string(actual));
}

template <typename Parameter, typename Settings, std::size_t Size,
    typename GetValue, typename SetValue, typename Reset>
void TestParameterFamily(const std::string &family, Settings settings,
    const std::array<gnc::ParameterMetadata<Parameter>, Size> &metadata,
    GetValue getValue, SetValue setValue, Reset reset) {
  for (std::size_t index = 0; index < Size; ++index) {
    const auto &descriptor = metadata[index];
    Require(static_cast<std::size_t>(descriptor.parameter) == index,
        family + " metadata order does not match its enum");
    Require(!descriptor.id.empty(), family + " parameter has no stable id");
    Require(!descriptor.displayName.empty(),
        family + " parameter has no display name");
    Require(descriptor.minimum <= descriptor.defaultValue
                && descriptor.defaultValue <= descriptor.maximum,
        family + " default is outside its bounds");
    Require(descriptor.increment > 0.0,
        family + " parameter has a non-positive edit increment");
    RequireNear(getValue(settings, descriptor.parameter),
        descriptor.defaultValue,
        family + " strongly typed default differs from metadata");

    for (std::size_t previous = 0; previous < index; ++previous) {
      Require(metadata[previous].id != descriptor.id,
          family + " contains duplicate parameter ids");
    }

    const double midpoint =
        descriptor.minimum + (descriptor.maximum - descriptor.minimum) * 0.4;
    Require(setValue(settings, descriptor.parameter, midpoint),
        family + " rejected a finite in-range value");
    RequireNear(getValue(settings, descriptor.parameter),
        midpoint,
        family + " set/get round trip changed the value");
  }

  const auto parameter = metadata.front().parameter;
  Require(setValue(settings, parameter, metadata.front().defaultValue),
      family + " could not restore the first parameter");
  const double beforeInvalid = getValue(settings, parameter);
  Require(
      !setValue(settings, parameter, std::numeric_limits<double>::quiet_NaN()),
      family + " accepted NaN");
  RequireNear(getValue(settings, parameter),
      beforeInvalid,
      family + " changed state after rejecting NaN");
  Require(
      !setValue(settings, parameter, std::numeric_limits<double>::infinity()),
      family + " accepted infinity");
  RequireNear(getValue(settings, parameter),
      beforeInvalid,
      family + " changed state after rejecting infinity");

  Require(setValue(settings, parameter, metadata.front().minimum - 1000.0),
      family + " rejected a finite under-range value");
  RequireNear(getValue(settings, parameter),
      metadata.front().minimum,
      family + " did not clamp its lower bound");
  Require(setValue(settings, parameter, metadata.front().maximum + 1000.0),
      family + " rejected a finite over-range value");
  RequireNear(getValue(settings, parameter),
      metadata.front().maximum,
      family + " did not clamp its upper bound");

  reset(settings);
  for (const auto &descriptor : metadata) {
    RequireNear(getValue(settings, descriptor.parameter),
        descriptor.defaultValue,
        family + " reset did not restore the algorithm default");
  }
}

void TestAllParameterFamilies() {
  TestParameterFamily("roll",
      gnc::Px4RollHoldReferenceSettings{},
      gnc::Px4RollHoldParameters,
      gnc::GetPx4RollHoldParameterValue,
      gnc::SetPx4RollHoldParameterValue,
      gnc::ResetPx4RollHoldParametersToDefaults);
  TestParameterFamily("pitch",
      gnc::Px4PitchHoldSettings{},
      gnc::Px4PitchHoldParameters,
      gnc::GetPx4PitchHoldParameterValue,
      gnc::SetPx4PitchHoldParameterValue,
      gnc::ResetPx4PitchHoldParametersToDefaults);
  TestParameterFamily("course",
      gnc::Px4CourseHoldSettings{},
      gnc::Px4CourseHoldParameters,
      gnc::GetPx4CourseHoldParameterValue,
      gnc::SetPx4CourseHoldParameterValue,
      gnc::ResetPx4CourseHoldParametersToDefaults);
  TestParameterFamily("yaw",
      gnc::Px4YawRateSettings{},
      gnc::Px4YawRateParameters,
      gnc::GetPx4YawRateParameterValue,
      gnc::SetPx4YawRateParameterValue,
      gnc::ResetPx4YawRateParametersToDefaults);
  TestParameterFamily("TECS",
      gnc::Px4TecsSettings{},
      gnc::Px4TecsParameters,
      gnc::GetPx4TecsParameterValue,
      gnc::TrySetPx4TecsParameterValue,
      gnc::ResetPx4TecsParametersToDefaults);
}

void TestResetPoliciesAreDistinct() {
  gnc::Px4RollHoldReferenceSettings roll;
  roll.trimAirspeedMps = 42.0;
  roll.directRollRateTestEnabled = true;
  roll.rateProportionalGain = 9.0;
  gnc::ResetPx4RollHoldParametersToDefaults(roll);
  RequireNear(roll.trimAirspeedMps,
      42.0,
      "algorithm-default reset changed the roll trim reference");
  Require(roll.directRollRateTestEnabled,
      "algorithm-default reset changed roll runtime test state");

  gnc::Px4PitchHoldSettings pitch;
  pitch.trimElevatorCommand = -0.2;
  pitch.rateProportionalGain = 9.0;
  gnc::ResetPx4PitchHoldParametersToDefaults(pitch);
  RequireNear(pitch.trimElevatorCommand,
      -0.2,
      "algorithm-default reset changed the pitch trim reference");

  gnc::Px4YawRateSettings yaw;
  yaw.setpointMode = gnc::Px4YawRateSetpointMode::CoordinatedTurn;
  yaw.trimRudderCommand = 0.1;
  yaw.rateProportionalGain = 9.0;
  gnc::ResetPx4YawRateParametersToDefaults(yaw);
  Require(yaw.setpointMode == gnc::Px4YawRateSetpointMode::CoordinatedTurn,
      "algorithm-default reset changed the yaw mode");
  RequireNear(yaw.trimRudderCommand,
      0.1,
      "algorithm-default reset changed the yaw trim reference");
}

void TestControllerBoundaryValidation() {
  gnc::Px4RollController roll;
  auto rollSettings = roll.GetSettings();
  rollSettings.timeConstantSec = std::numeric_limits<double>::quiet_NaN();
  rollSettings.rateProportionalGain = 100.0;
  roll.SetSettings(rollSettings);
  RequireNear(roll.GetSettings().timeConstantSec,
      gnc::GetPx4RollHoldParameterMetadata(
          gnc::Px4RollHoldParameter::TimeConstant)
          .defaultValue,
      "roll controller did not replace NaN with its default");
  RequireNear(roll.GetSettings().rateProportionalGain,
      gnc::GetPx4RollHoldParameterMetadata(
          gnc::Px4RollHoldParameter::RateProportionalGain)
          .maximum,
      "roll controller did not clamp an out-of-range setting");

  gnc::Px4PitchController pitch;
  auto pitchSettings = pitch.GetSettings();
  pitchSettings.rateIntegralGain = std::numeric_limits<double>::infinity();
  pitch.SetSettings(pitchSettings);
  RequireNear(pitch.GetSettings().rateIntegralGain,
      gnc::GetPx4PitchHoldParameterMetadata(
          gnc::Px4PitchHoldParameter::RateIntegralGain)
          .defaultValue,
      "pitch controller did not replace infinity with its default");

  gnc::Px4CourseController course;
  auto courseSettings = course.GetSettings();
  courseSettings.guidanceDampingRatio = -100.0;
  course.SetSettings(courseSettings);
  RequireNear(course.GetSettings().guidanceDampingRatio,
      gnc::GetPx4CourseHoldParameterMetadata(
          gnc::Px4CourseHoldParameter::GuidanceDamping)
          .minimum,
      "course controller did not clamp an out-of-range setting");

  gnc::Px4YawRateController yaw;
  auto yawSettings = yaw.GetSettings();
  yawSettings.rateFeedForwardGain = std::numeric_limits<double>::quiet_NaN();
  yaw.SetSettings(yawSettings);
  RequireNear(yaw.GetSettings().rateFeedForwardGain,
      gnc::GetPx4YawRateParameterMetadata(
          gnc::Px4YawRateParameter::RateFeedForwardGain)
          .defaultValue,
      "yaw controller did not replace NaN with its default");

  gnc::Px4TecsController tecs;
  auto tecsSettings = tecs.GetSettings();
  tecsSettings.throttleDampingGain = std::numeric_limits<double>::infinity();
  tecs.SetSettings(tecsSettings);
  RequireNear(tecs.GetSettings().throttleDampingGain,
      gnc::GetPx4TecsParameterMetadata(
          gnc::Px4TecsParameter::ThrottleDampingGain)
          .defaultValue,
      "TECS did not replace infinity with its default");
}

void TestDisplayConversion() {
  const auto &pitchLimit =
      gnc::GetPx4TecsParameterMetadata(gnc::Px4TecsParameter::MaximumPitch);
  RequireNear(gnc::ToParameterDisplayValue(pitchLimit, math::DegToRad(15.0)),
      15.0,
      "radian parameter did not convert to degrees at the display boundary");
  RequireNear(gnc::FromParameterDisplayValue(pitchLimit, 15.0),
      math::DegToRad(15.0),
      "degree display value did not convert to radians at the input boundary");
  RequireNear(gnc::GetParameterDisplayIncrement(pitchLimit),
      math::RadToDeg(pitchLimit.increment),
      "display increment did not use the parameter unit conversion");
}

void TestC172xProfilePreservesTuning() {
  const auto &profile = gnc::GetC172xPx4ControlProfile();
  const auto copy = gnc::MakeC172xPx4ControlProfile();
  Require(profile.aircraftId == "c172x" && copy.aircraftId == "c172x",
      "C172x profile identity changed");
  RequireNear(profile.roll.rateProportionalGain,
      1.9,
      "C172x roll tuning changed");
  RequireNear(profile.pitch.rateProportionalGain,
      4.5,
      "C172x pitch tuning changed");
  RequireNear(profile.course.guidancePeriodSec,
      10.0,
      "C172x course tuning changed");
  RequireNear(profile.yaw.rateProportionalGain,
      0.8,
      "C172x yaw tuning changed");
  RequireNear(profile.yaw.sideslipToYawRateGain,
      8.0,
      "C172x sideslip tuning changed");
  RequireNear(profile.tecs.altitudeErrorGain,
      0.10,
      "C172x TECS altitude tuning changed");
  RequireNear(profile.tecs.airspeedErrorGain,
      0.40,
      "C172x TECS airspeed tuning changed");
  RequireNear(copy.tecs.throttleIntegralGain,
      profile.tecs.throttleIntegralGain,
      "C172x profile copy differs from the central profile");
}
} // namespace

int main() {
  TestAllParameterFamilies();
  TestResetPoliciesAreDistinct();
  TestControllerBoundaryValidation();
  TestDisplayConversion();
  TestC172xProfilePreservesTuning();
  std::cout << "Parameter tests passed\n";
  return 0;
}
