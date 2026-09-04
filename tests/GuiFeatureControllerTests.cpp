#include "gui/features/gnc/GNCController.hpp"
#include "gui/features/linearization/LinearizationController.hpp"
#include "gui/features/simulation/ScenarioController.hpp"
#include "gui/features/simulation/SimController.hpp"
#include "common/math/Math.hpp"
#include "messaging/MessageBus.hpp"
#include "messaging/SimMessageClient.hpp"
#include "messaging/SimMessages.hpp"

#include <array>
#include <cassert>
#include <limits>
#include <string_view>

namespace {
namespace messaging = app::messaging;

void TestSimulationEventsPublishCommandsAndUpdateChildModel() {
  messaging::MessageBus bus;
  app::SimMessageClient client(bus);
  gui::SimController controller(client);

  int startCount = 0;
  double requestedHz = 0.0;
  bool maximumSpeed = false;
  auto startSubscription = bus.Subscribe<messaging::SimStartCommand>(
      [&startCount](const auto &) { ++startCount; });
  auto rateSubscription = bus.Subscribe<messaging::SimRateCommand>(
      [&requestedHz](const auto &command) { requestedHz = command.hz; });
  auto maximumSubscription =
      bus.Subscribe<messaging::SimMaximumSpeedCommand>(
          [&maximumSpeed](
              const auto &command) { maximumSpeed = command.enabled; });

  controller.OnEvent(gui::SimStartRequested{});
  controller.OnEvent(gui::SimRateChanged{120.0});
  controller.OnEvent(gui::MaximumSimulationSpeedChanged{true});
  assert(startCount == 1);
  assert(requestedHz == 120.0);
  assert(maximumSpeed);

  sim::SimSnapshot snapshot;
  snapshot.defaultInitialCondition.altitudeAslM = 1200.0;
  controller.Synchronize(snapshot);
  controller.OnEvent({gui::InitialConditionField::AltitudeAslM, 1500.0});
  assert(controller.GetInitialConditionModel().pending.altitudeAslM == 1500.0);
  controller.OnEvent({gui::InitialConditionField::LatitudeDeg, 45.0});
  assert(std::abs(controller.GetInitialConditionModel().pending.latitudeRad
                  - math::DegToRad(45.0))
         < 1.0e-12);
  controller.OnEvent(
      {gui::InitialConditionField::CalibratedAirspeedMps, 41.16});
  assert(controller.GetInitialConditionModel()
             .pending.calibratedAirspeedMps
         == 41.16);
}

void TestPlaybackToggleSelectsStartOrStopFromRuntimeState() {
  messaging::MessageBus bus;
  app::SimMessageClient client(bus);
  gui::SimController controller(client);
  int startCount = 0;
  int stopCount = 0;
  auto startSubscription = bus.Subscribe<messaging::SimStartCommand>(
      [&startCount](const auto &) { ++startCount; });
  auto stopSubscription = bus.Subscribe<messaging::SimStopCommand>(
      [&stopCount](const auto &) { ++stopCount; });

  controller.OnEvent(gui::SimPlaybackToggled{});
  assert(startCount == 1);
  assert(stopCount == 0);

  sim::SimStatus status;
  status.executionState = sim::SimExecutionState::Running;
  bus.Publish(messaging::SimStatusEvent{.status = status});
  controller.OnEvent(gui::SimPlaybackToggled{});
  assert(startCount == 1);
  assert(stopCount == 1);

  status.executionState = sim::SimExecutionState::Paused;
  bus.Publish(messaging::SimStatusEvent{.status = status});
  controller.OnEvent(gui::SimPlaybackToggled{});
  assert(stopCount == 2);
}

void TestGNCEventsUpdateModelAndPublishCompleteConfig() {
  messaging::MessageBus bus;
  app::SimMessageClient client(bus);
  gui::GNCController controller(client);
  sim::SimSnapshot snapshot;
  snapshot.primary.available = true;
  snapshot.primaryAutopilot.available = true;
  controller.Synchronize(snapshot);

  controller.OnEvent(
      gui::PrimaryRollHoldValueChanged{gui::PrimaryRollHoldField::TargetDeg,
          12.5});
  assert(controller.GetModel().primaryAutopilot.rollTargetDeg == 12.5);

  sim::PrimaryRollHoldConfig published;
  auto subscription = bus.Subscribe<messaging::PrimaryRollHoldConfigCommand>(
      [&published](const auto &command) { published = command.config; });
  controller.PublishConfiguration(snapshot);
  assert(published.targetRollRad != 0.0);
  assert(published.rollAngleProportionalGain
         == controller.GetModel().primaryAutopilot.rollAngleProportionalGain);
}

void TestTecsCapturePreservesSiValues() {
  constexpr double AltitudeAglM = math::FeetToMeters(1200.0);
  constexpr double CalibratedAirspeedMps = math::KnotsToMetersPerSecond(80.0);
  static_assert(AltitudeAglM == 365.76);
  static_assert(CalibratedAirspeedMps > 41.15 && CalibratedAirspeedMps < 41.16);

  gui::BaselineAutopilotPanelState state;
  gui::TecsController controller(state);
  controller.OnEvent(gui::BaselineTecsAltitudeCaptureRequested{AltitudeAglM});
  controller.OnEvent(
      gui::BaselineTecsAirspeedCaptureRequested{CalibratedAirspeedMps});
  assert(state.tecsTargetAltitudeM == AltitudeAglM);
  assert(state.tecsTargetAirspeedMps == CalibratedAirspeedMps);

  controller.OnEvent(gui::BaselineTecsAltitudeCaptureRequested{
      std::numeric_limits<double>::quiet_NaN()});
  controller.OnEvent(gui::BaselineTecsAirspeedCaptureRequested{-1.0});
  assert(state.tecsTargetAltitudeM == AltitudeAglM);
  assert(state.tecsTargetAirspeedMps == CalibratedAirspeedMps);
}

void TestPx4FeatureControllersOwnStateValidationAndViewState() {
  gui::AutopilotPanelState primary;
  gui::BaselineAutopilotPanelState baseline;
  gui::ExperimentalController experimental(primary);
  gui::Px4AttitudeController controller(baseline);

  experimental.OnEvent(
      gui::PrimaryRollHoldValueChanged{gui::PrimaryRollHoldField::TargetDeg,
          7.5});
  controller.OnEvent(gui::BaselineRollHoldValueChanged{
      gui::BaselineRollHoldField::CourseHoldEnabled,
      1.0});
  assert(primary.rollTargetDeg == 7.5);
  assert(baseline.courseHold);
  assert(baseline.rollHold);

  experimental.OnEvent(gui::ExperimentalViewStateChanged{false});
  controller.OnEvent(gui::Px4AttitudeViewStateChanged{true, false, true, false});
  assert(!primary.rollHoldParametersOpen);
  assert(baseline.px4RollTuningOpen);
  assert(!baseline.px4RollDiagnosticsOpen);
  assert(baseline.px4PitchTuningOpen);
  assert(!baseline.px4PitchDiagnosticsOpen);

  experimental.OnEvent(
      gui::PrimaryRollHoldValueChanged{gui::PrimaryRollHoldField::TargetDeg,
          std::numeric_limits<double>::infinity()});
  assert(primary.rollTargetDeg == 7.5);
}

void TestTrimRequestStateUsesSi() {
  messaging::MessageBus bus;
  app::SimMessageClient client(bus);
  gui::GNCController controller(client);
  gnc::TrimRequest publishedRequest;
  bool publishedFromCurrentState = false;
  auto subscription = bus.Subscribe<messaging::TrimCommand>(
      [&publishedRequest, &publishedFromCurrentState](const auto &command) {
        publishedRequest = command.request;
        publishedFromCurrentState = command.fromCurrentState;
      });
  controller.OnEvent(
      gui::TrimRequestValueChanged{gui::TrimRequestField::CalibratedAirspeedMps,
          math::KnotsToMetersPerSecond(80.0)});
  controller.OnEvent(
      gui::TrimRequestValueChanged{gui::TrimRequestField::AltitudeAslM,
          math::FeetToMeters(1200.0)});
  controller.OnEvent(
      gui::TrimRequestValueChanged{gui::TrimRequestField::FlightPathAngleRad,
          math::DegToRad(3.0)});
  controller.OnEvent(
      gui::TrimRequestValueChanged{gui::TrimRequestField::AltitudeAslM,
          std::numeric_limits<double>::quiet_NaN()});

  const gnc::TrimRequest &request = controller.GetModel().trimRequest;
  assert(request.calibratedAirspeedMps == math::KnotsToMetersPerSecond(80.0));
  assert(request.altitudeAslM == 365.76);
  assert(request.flightPathAngleRad == math::DegToRad(3.0));

  controller.OnEvent(gui::TrimExecutionRequested{true});
  assert(publishedRequest.calibratedAirspeedMps
         == math::KnotsToMetersPerSecond(80.0));
  assert(publishedFromCurrentState);
  assert(!controller.GetModel().trimInProgress);
}

void TestBaselinePx4TuningUsesSharedMetadata() {
  struct ExpectedParameter {
    std::string_view name;
    double minimum;
    double maximum;
    double defaultValue;
    double increment;
  };
  constexpr std::array<ExpectedParameter, 7> ExpectedParameters{{
      {"FW_R_TC", 0.2, 1.0, 0.4, 0.05},
      {"FW_R_RMAX", 0.0, 180.0, 70.0, 0.5},
      {"FW_RR_P", 0.0, 10.0, 1.9, 0.005},
      {"FW_RR_I", 0.0, 10.0, 0.25, 0.01},
      {"FW_RR_D", 0.0, 10.0, 0.0, 0.005},
      {"FW_RR_FF", 0.0, 10.0, 1.2, 0.05},
      {"FW_RR_IMAX", 0.0, 1.0, 0.2, 0.05},
  }};

  assert(gnc::Px4RollHoldParameters.size() == ExpectedParameters.size());
  for (std::size_t index = 0; index < ExpectedParameters.size(); ++index) {
    const auto &metadata = gnc::Px4RollHoldParameters[index];
    const auto &expected = ExpectedParameters[index];
    assert(metadata.id == expected.name);
    assert(metadata.minimum == expected.minimum);
    assert(metadata.maximum == expected.maximum);
    assert(metadata.defaultValue == expected.defaultValue);
    assert(metadata.increment == expected.increment);
  }

  constexpr std::array<ExpectedParameter, 8> ExpectedPitchParameters{{
      {"FW_P_TC", 0.2, 1.0, 0.2, 0.05},
      {"FW_P_RMAX_POS", 0.0, 180.0, 14.0, 0.5},
      {"FW_P_RMAX_NEG", 0.0, 180.0, 10.0, 0.5},
      {"FW_PR_P", 0.0, 10.0, 4.5, 0.005},
      {"FW_PR_I", 0.0, 10.0, 4.5, 0.005},
      {"FW_PR_D", 0.0, 10.0, 0.0, 0.005},
      {"FW_PR_FF", -10.0, 10.0, 1.2, 0.05},
      {"FW_PR_IMAX", 0.0, 1.0, 0.4, 0.05},
  }};
  assert(gnc::Px4PitchHoldParameters.size() == ExpectedPitchParameters.size());
  for (std::size_t index = 0; index < ExpectedPitchParameters.size(); ++index) {
    const auto &metadata = gnc::Px4PitchHoldParameters[index];
    const auto &expected = ExpectedPitchParameters[index];
    assert(metadata.id == expected.name);
    assert(metadata.minimum == expected.minimum);
    assert(metadata.maximum == expected.maximum);
    assert(metadata.defaultValue == expected.defaultValue);
    assert(metadata.increment == expected.increment);
  }

  messaging::MessageBus bus;
  app::SimMessageClient client(bus);
  gui::GNCController controller(client);

  assert(gui::BaselinePx4RollHoldParameterBindings.size() == 7);
  for (const auto &binding : gui::BaselinePx4RollHoldParameterBindings) {
    const auto &metadata =
        gnc::GetPx4RollHoldParameterMetadata(binding.parameter);

    controller.OnEvent(gui::BaselineRollHoldValueChanged{binding.field,
        metadata.minimum - 1000.0});
    assert(controller.GetModel().baselineAutopilot.*(binding.value)
           == metadata.minimum);

    controller.OnEvent(gui::BaselineRollHoldValueChanged{binding.field,
        metadata.maximum + 1000.0});
    assert(controller.GetModel().baselineAutopilot.*(binding.value)
           == metadata.maximum);
  }

  assert(gui::BaselinePx4PitchHoldParameterBindings.size() == 8);
  for (const auto &binding : gui::BaselinePx4PitchHoldParameterBindings) {
    const auto &metadata =
        gnc::GetPx4PitchHoldParameterMetadata(binding.parameter);

    controller.OnEvent(gui::BaselineRollHoldValueChanged{binding.field,
        metadata.minimum - 1000.0});
    assert(controller.GetModel().baselineAutopilot.*(binding.value)
           == metadata.minimum);

    controller.OnEvent(gui::BaselineRollHoldValueChanged{binding.field,
        metadata.maximum + 1000.0});
    assert(controller.GetModel().baselineAutopilot.*(binding.value)
           == metadata.maximum);
  }
  controller.OnEvent(gui::BaselineRollHoldValueChanged{
      gui::BaselineRollHoldField::PitchHoldEnabled,
      1.0});
  controller.OnEvent(gui::BaselineRollHoldValueChanged{
      gui::BaselineRollHoldField::TargetPitchDeg,
      100.0});
  assert(controller.GetModel().baselineAutopilot.pitchHold);
  assert(controller.GetModel().baselineAutopilot.pitchTargetDeg == 90.0);
  controller.OnEvent(gui::BaselineRollHoldValueChanged{
      gui::BaselineRollHoldField::TargetPitchDeg,
      -4.5});

  assert(gnc::Px4TecsParameters.size()
         == static_cast<std::size_t>(gnc::Px4TecsParameter::Count));
  controller.OnEvent(
      gui::BaselineTecsValueChanged{gui::BaselineTecsField::Enabled, 1.0});
  controller.OnEvent(
      gui::BaselineTecsValueChanged{gui::BaselineTecsField::TargetAltitudeM,
          354.8});
  controller.OnEvent(
      gui::BaselineTecsValueChanged{gui::BaselineTecsField::TargetAirspeedMps,
          44.0});
  controller.OnEvent(
      gui::BaselineTecsParameterChanged{gnc::Px4TecsParameter::MaximumClimbRate,
          3.25});
  controller.OnEvent(
      gui::BaselineTecsParameterChanged{gnc::Px4TecsParameter::MinimumPitch,
          math::DegToRad(-12.0)});
  assert(controller.GetModel().baselineAutopilot.tecs);
  assert(controller.GetModel().baselineAutopilot.tecsTargetAltitudeM == 354.8);
  assert(controller.GetModel().baselineAutopilot.tecsTargetAirspeedMps == 44.0);
  assert(
      controller.GetModel().baselineAutopilot.tecsSettings.maximumClimbRateMps
      == 3.25);
  assert(controller.GetModel().baselineAutopilot.tecsSettings.minimumPitchRad
         == math::DegToRad(-12.0));

  assert(gui::BaselinePx4CourseHoldParameterBindings.size() == 4);
  for (const auto &binding : gui::BaselinePx4CourseHoldParameterBindings) {
    const auto &metadata =
        gnc::GetPx4CourseHoldParameterMetadata(binding.parameter);
    controller.OnEvent(gui::BaselineRollHoldValueChanged{binding.field,
        metadata.maximum + 1000.0});
    assert(controller.GetModel().baselineAutopilot.*(binding.value)
           == metadata.maximum);
  }

  assert(gui::BaselinePx4YawRateParameterBindings.size()
         == static_cast<std::size_t>(gnc::Px4YawRateParameter::Count));
  for (const auto &binding : gui::BaselinePx4YawRateParameterBindings) {
    const auto &metadata =
        gnc::GetPx4YawRateParameterMetadata(binding.parameter);
    controller.OnEvent(gui::BaselineRollHoldValueChanged{binding.field,
        metadata.minimum - 1000.0});
    assert(controller.GetModel().baselineAutopilot.*(binding.value)
           == metadata.minimum);
    controller.OnEvent(gui::BaselineRollHoldValueChanged{binding.field,
        metadata.maximum + 1000.0});
    assert(controller.GetModel().baselineAutopilot.*(binding.value)
           == metadata.maximum);
  }
  controller.OnEvent(gui::BaselineRollHoldValueChanged{
      gui::BaselineRollHoldField::CourseHoldEnabled,
      1.0});
  controller.OnEvent(gui::BaselineRollHoldValueChanged{
      gui::BaselineRollHoldField::TargetCourseDeg,
      -179.0});
  assert(controller.GetModel().baselineAutopilot.courseHold);
  assert(controller.GetModel().baselineAutopilot.rollHold);
  assert(controller.GetModel().baselineAutopilot.targetCourseDeg == -179.0);

  controller.OnEvent(gui::BaselineRollHoldValueChanged{
      gui::BaselineRollHoldField::RateProportionalGain,
      0.1234});
  assert(controller.GetModel().baselineAutopilot.px4RollRateProportionalGain
         == 0.123);

  controller.OnEvent(gui::BaselineRollHoldValueChanged{
      gui::BaselineRollHoldField::DirectRollRateTestEnabled,
      1.0});
  controller.OnEvent(gui::BaselineRollHoldValueChanged{
      gui::BaselineRollHoldField::DirectRollRateCommandDegPerSec,
      5.0});
  assert(controller.GetModel().baselineAutopilot.directRollRateTestEnabled);
  assert(controller.GetModel().baselineAutopilot.directRollRateCommandDegPerSec
         == 5.0);

  controller.OnEvent(gui::BaselineRollHoldValueChanged{
      gui::BaselineRollHoldField::YawRateControlEnabled,
      1.0});
  controller.OnEvent(gui::BaselineRollHoldValueChanged{
      gui::BaselineRollHoldField::CoordinatedTurnEnabled,
      1.0});
  controller.OnEvent(gui::BaselineRollHoldValueChanged{
      gui::BaselineRollHoldField::YawRateProportionalGain,
      0.8});
  controller.OnEvent(gui::BaselineRollHoldValueChanged{
      gui::BaselineRollHoldField::SideslipToYawRateGain,
      8.0});
  assert(controller.GetModel().baselineAutopilot.yawRateControlEnabled);
  assert(controller.GetModel().baselineAutopilot.coordinatedTurnEnabled);
  assert(controller.GetModel().baselineAutopilot.px4YawRateProportionalGain
         == 0.8);
  assert(controller.GetModel().baselineAutopilot.sideslipToYawRateGain == 8.0);

  sim::BaselineRollHoldConfig published;
  auto subscription = bus.Subscribe<messaging::BaselineRollHoldConfigCommand>(
      [&published](const auto &command) { published = command.config; });
  sim::SimSnapshot snapshot;
  snapshot.baseline = sim::SimInstanceSnapshot{.available = true};
  snapshot.baselineAutopilot = sim::AutopilotSnapshot{.available = true};
  controller.PublishConfiguration(snapshot);
  assert(published.directRollRateTestEnabled);
  assert(published.directRollRateCommandRadPerSec == math::DegToRad(5.0));
  assert(published.yawRateControlEnabled);
  assert(published.coordinatedTurnEnabled);
  assert(published.maximumYawRateRadPerSec == math::DegToRad(90.0));
  assert(published.yawRateProportionalGain == 0.8);
  assert(published.yawIntegratorLimit == 1.0);
  assert(published.sideslipToYawRateGain == 8.0);
  assert(published.courseHoldEnabled);
  assert(published.targetCourseRad == math::DegToRad(-179.0));
  assert(published.pitchHoldEnabled);
  assert(published.targetPitchRad == math::DegToRad(-4.5));
  assert(published.pitchTimeConstantSec == 1.0);
  assert(published.maximumPositivePitchRateRadPerSec == math::DegToRad(180.0));
  assert(published.maximumNegativePitchRateRadPerSec == math::DegToRad(180.0));
  assert(published.pitchRateFeedForwardGain == 10.0);
  assert(published.pitchIntegratorLimit == 1.0);
  assert(published.tecsEnabled);
  assert(published.targetAltitudeM == 354.8);
  assert(published.targetAirspeedMps == 44.0);
  assert(published.tecsSettings.maximumClimbRateMps == 3.25);
  assert(published.tecsSettings.minimumPitchRad == math::DegToRad(-12.0));

  controller.OnEvent(gui::BaselineRollHoldTuningResetRequested{});
  assert(!controller.GetModel().baselineAutopilot.directRollRateTestEnabled);
  assert(controller.GetModel().baselineAutopilot.directRollRateCommandDegPerSec
         == 0.0);
  assert(!controller.GetModel().baselineAutopilot.yawRateControlEnabled);
  assert(controller.GetModel().baselineAutopilot.coordinatedTurnEnabled);
  assert(controller.GetModel().baselineAutopilot.px4YawRateProportionalGain
         == 0.8);
  assert(controller.GetModel().baselineAutopilot.sideslipToYawRateGain == 8.0);
  const auto &c172xProfile = gnc::GetC172xPx4ControlProfile();
  for (const auto &binding : gui::BaselinePx4YawRateParameterBindings) {
    assert(controller.GetModel().baselineAutopilot.*(binding.value)
           == gnc::GetPx4YawRateParameterValue(c172xProfile.yaw,
               binding.parameter));
  }
  for (const auto &binding : gui::BaselinePx4RollHoldParameterBindings) {
    const auto &metadata =
        gnc::GetPx4RollHoldParameterMetadata(binding.parameter);
    assert(controller.GetModel().baselineAutopilot.*(binding.value)
           == metadata.defaultValue);
  }

  controller.OnEvent(gui::BaselinePitchHoldTuningResetRequested{});
  for (const auto &binding : gui::BaselinePx4PitchHoldParameterBindings) {
    const auto &metadata =
        gnc::GetPx4PitchHoldParameterMetadata(binding.parameter);
    assert(controller.GetModel().baselineAutopilot.*(binding.value)
           == metadata.defaultValue);
  }
  const double trimThrottle =
      controller.GetModel().baselineAutopilot.tecsSettings.trimThrottle;
  controller.OnEvent(gui::BaselineTecsTuningResetRequested{});
  assert(
      controller.GetModel().baselineAutopilot.tecsSettings.maximumClimbRateMps
      == gnc::Px4TecsSettings{}.maximumClimbRateMps);
  assert(controller.GetModel().baselineAutopilot.tecsSettings.trimThrottle
         == trimThrottle);
}

void TestLinearizationEventPublishesCommand() {
  messaging::MessageBus bus;
  app::SimMessageClient client(bus);
  gui::LinearizationController controller(client);
  bool automatic = false;
  auto subscription = bus.Subscribe<messaging::LinearizationConfigCommand>(
      [&automatic](const auto &command) {
        automatic = command.automaticUpdatesEnabled;
      });

  controller.OnEvent(gui::AutomaticLinearizationChanged{true});
  controller.OnEvent(gui::LinearizationValueTransformChanged{
      gui::LinearizationValueTransform::SignedLog10});

  assert(automatic);
  assert(controller.GetModel().valueTransform
         == gui::LinearizationValueTransform::SignedLog10);
}

void TestScenarioChildUpdatesDraftAndEmitsLaunchIntent() {
  bool launchReceived = false;
  gui::ScenarioController *controllerPtr = nullptr;
  gui::ScenarioController controller({},
      gui::architecture::EventSink<gui::ScenarioLaunchRequested>{
          [&launchReceived, &controllerPtr](
              const gui::ScenarioLaunchRequested &event) {
            launchReceived =
                event.request.scenario.events.front().command.rollRad
                    == math::DegToRad(14.0);
            controllerPtr->OnEvent(gui::ScenarioApplyCompleted{
                .succeeded = true,
            });
          }});
  controllerPtr = &controller;
  sim::SimScenario draft = controller.GetModel().draft;
  draft.events.front().command.rollRad = math::DegToRad(14.0);

  controller.OnEvent(gui::ScenarioDraftChanged{draft});
  assert(controller.Apply());

  assert(controller.GetModel().draft.events.front().command.rollRad
         == math::DegToRad(14.0));
  assert(launchReceived);
}
} // namespace

int main() {
  TestSimulationEventsPublishCommandsAndUpdateChildModel();
  TestPlaybackToggleSelectsStartOrStopFromRuntimeState();
  TestGNCEventsUpdateModelAndPublishCompleteConfig();
  TestTecsCapturePreservesSiValues();
  TestPx4FeatureControllersOwnStateValidationAndViewState();
  TestTrimRequestStateUsesSi();
  TestBaselinePx4TuningUsesSharedMetadata();
  TestLinearizationEventPublishesCommand();
  TestScenarioChildUpdatesDraftAndEmitsLaunchIntent();
  return 0;
}
