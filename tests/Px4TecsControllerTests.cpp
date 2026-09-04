#include "sim/gnc/tecs/Px4TecsController.hpp"

#include <algorithm>
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

void RequireNear(double actual, double expected, double tolerance,
    const std::string &message) {
  Require(std::abs(actual - expected) <= tolerance,
      message + ": expected " + std::to_string(expected) + ", got "
          + std::to_string(actual));
}

gnc::Px4TecsInput NominalInput() {
  return {
      .altitudeM = 300.0,
      .verticalSpeedMps = 0.0,
      .calibratedAirspeedMps = 41.0,
      .targetAltitudeM = 300.0,
      .targetAirspeedMps = 41.0,
      .currentPitchRad = 0.04,
      .currentThrottle = 0.47,
      .gravityMps2 = 9.80665,
      .dtSec = 1.0 / 120.0,
  };
}

void TestParameterMetadataAndSettingsUseSi() {
  const gnc::Px4TecsSettings settings;
  const auto &minimumPitch =
      gnc::GetPx4TecsParameterMetadata(gnc::Px4TecsParameter::MinimumPitch);
  const auto &pitchSlew =
      gnc::GetPx4TecsParameterMetadata(gnc::Px4TecsParameter::PitchSlewRate);
  Require(gnc::GetUnitSymbol(minimumPitch.unit) == "rad",
      "TECS pitch metadata was not radians");
  Require(gnc::GetUnitSymbol(pitchSlew.unit) == "rad/s",
      "TECS pitch-slew metadata was not radians per second");
  RequireNear(minimumPitch.defaultValue,
      math::DegToRad(-10.0),
      Tolerance,
      "TECS minimum-pitch default changed during SI conversion");
  RequireNear(settings.minimumPitchRad,
      minimumPitch.defaultValue,
      Tolerance,
      "TECS settings did not store minimum pitch in radians");
  RequireNear(gnc::GetPx4TecsParameterValue(settings,
                  gnc::Px4TecsParameter::PitchSlewRate),
      math::DegToRad(8.0),
      Tolerance,
      "TECS parameter access did not return SI pitch slew");
}

void TestBumplessInitializationAndReset() {
  gnc::Px4TecsController controller;
  const gnc::Px4TecsInput input = NominalInput();
  const gnc::Px4TecsOutput first = controller.Update(input);
  Require(first.valid, "TECS rejected a valid initialization input");
  RequireNear(first.targetPitchRad,
      input.currentPitchRad,
      Tolerance,
      "TECS initialization changed pitch");
  RequireNear(first.targetThrottle,
      input.currentThrottle,
      Tolerance,
      "TECS initialization changed throttle");

  controller.Reset();
  const gnc::Px4TecsOutput afterReset = controller.Update(input);
  RequireNear(afterReset.targetPitchRad,
      input.currentPitchRad,
      Tolerance,
      "TECS reset did not restore bumpless pitch initialization");
  RequireNear(afterReset.targetThrottle,
      input.currentThrottle,
      Tolerance,
      "TECS reset did not restore bumpless throttle initialization");
}

void TestSpecificEnergyCalculationAndReferenceRateLimit() {
  gnc::Px4TecsController controller;
  gnc::Px4TecsInput input = NominalInput();
  input.altitudeM = 365.76;
  input.calibratedAirspeedMps = 41.16;
  input.targetAltitudeM = input.altitudeM;
  input.targetAirspeedMps = input.calibratedAirspeedMps;
  controller.Update(input);
  input.targetAltitudeM += 50.0;
  input.targetAirspeedMps += 3.0;
  const gnc::Px4TecsOutput output = controller.Update(input);
  Require(output.valid, "TECS rejected a valid energy input");

  const auto &diagnostics = controller.GetDiagnostics();
  const double expectedInternalAltitude =
      input.altitudeM
      + controller.GetSettings().maximumClimbRateMps * input.dtSec;
  RequireNear(diagnostics.internalAltitudeSetpointM,
      expectedInternalAltitude,
      Tolerance,
      "TECS altitude reference did not respect climb-rate limit");
  RequireNear(diagnostics.potentialEnergy,
      input.gravityMps2 * input.altitudeM,
      Tolerance,
      "TECS potential energy calculation changed");
  RequireNear(diagnostics.kineticEnergy,
      0.5 * input.calibratedAirspeedMps * input.calibratedAirspeedMps,
      Tolerance,
      "TECS kinetic energy calculation changed");
  RequireNear(diagnostics.totalEnergyError,
      diagnostics.potentialEnergyError + diagnostics.kineticEnergyError,
      Tolerance,
      "TECS total-energy error is not SPE + SKE");
  RequireNear(diagnostics.energyBalanceError,
      diagnostics.potentialEnergyError - diagnostics.kineticEnergyError,
      Tolerance,
      "TECS balance error is not SPE - SKE");
}

void TestSettingsAndOutputClamps() {
  gnc::Px4TecsController controller;
  gnc::Px4TecsSettings settings = controller.GetSettings();
  settings.minimumPitchRad = 1.0;
  settings.maximumPitchRad = -1.0;
  settings.minimumThrottle = 0.8;
  settings.maximumThrottle = 0.2;
  settings.trimThrottle = 2.0;
  settings.minimumAirspeedMps = 80.0;
  settings.maximumAirspeedMps = 20.0;
  controller.SetSettings(settings);
  const gnc::Px4TecsSettings &actual = controller.GetSettings();
  Require(actual.minimumPitchRad <= actual.maximumPitchRad,
      "TECS pitch limits were not ordered");
  Require(actual.minimumThrottle <= actual.maximumThrottle,
      "TECS throttle limits were not ordered");
  Require(actual.trimThrottle >= actual.minimumThrottle
              && actual.trimThrottle <= actual.maximumThrottle,
      "TECS trim throttle escaped throttle limits");
  Require(actual.minimumAirspeedMps <= actual.maximumAirspeedMps,
      "TECS airspeed limits were not ordered");

  gnc::Px4TecsInput input = NominalInput();
  controller.Update(input);
  input.targetAltitudeM += 1000.0;
  for (int index = 0; index < 2000; ++index) {
    const gnc::Px4TecsOutput output = controller.Update(input);
    Require(output.targetPitchRad >= actual.minimumPitchRad
                && output.targetPitchRad <= actual.maximumPitchRad,
        "TECS pitch output escaped its limits");
    Require(output.targetThrottle >= actual.minimumThrottle
                && output.targetThrottle <= actual.maximumThrottle,
        "TECS throttle output escaped its limits");
  }
}

void TestInvalidInputAndDtHandling() {
  gnc::Px4TecsController controller;
  gnc::Px4TecsInput input = NominalInput();
  input.dtSec = 0.0;
  Require(!controller.Update(input).valid, "TECS accepted zero dt");
  input = NominalInput();
  input.calibratedAirspeedMps = std::numeric_limits<double>::quiet_NaN();
  Require(!controller.Update(input).valid, "TECS accepted NaN airspeed");
}

void TestAntiWindupAndUnderspeedProtection() {
  gnc::Px4TecsController controller;
  gnc::Px4TecsInput input = NominalInput();
  controller.Update(input);
  input.targetAltitudeM += 1000.0;
  for (int index = 0; index < 4000; ++index) {
    controller.Update(input);
  }
  const auto &highEnergy = controller.GetDiagnostics();
  const auto &settings = controller.GetSettings();
  Require(highEnergy.throttleIntegralTerm
              <= settings.maximumThrottle - settings.trimThrottle + Tolerance,
      "TECS throttle anti-windup limit was exceeded");
  Require(highEnergy.pitchIntegralTerm <= settings.maximumPitchRad + Tolerance,
      "TECS pitch anti-windup limit was exceeded");

  controller.Reset();
  input = NominalInput();
  input.calibratedAirspeedMps = settings.minimumAirspeedMps * 0.85;
  input.targetAirspeedMps = settings.minimumAirspeedMps;
  input.targetAltitudeM += 100.0;
  controller.Update(input);
  for (int index = 0; index < 600; ++index) {
    controller.Update(input);
  }
  const auto &underspeed = controller.GetDiagnostics();
  Require(underspeed.underspeedProtectionActive,
      "TECS underspeed protection did not activate");
  RequireNear(underspeed.targetThrottle,
      settings.maximumThrottle,
      1.0e-6,
      "TECS underspeed protection did not demand maximum throttle");
  Require(underspeed.targetPitchRad < 0.0,
      "TECS underspeed protection continued to demand nose-up pitch");
}
} // namespace

int main() {
  TestParameterMetadataAndSettingsUseSi();
  TestBumplessInitializationAndReset();
  TestSpecificEnergyCalculationAndReferenceRateLimit();
  TestSettingsAndOutputClamps();
  TestInvalidInputAndDtHandling();
  TestAntiWindupAndUnderspeedProtection();
  return 0;
}
