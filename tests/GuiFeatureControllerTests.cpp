#include "gui/features/gnc/GNCController.hpp"
#include "gui/features/linearization/LinearizationController.hpp"
#include "gui/features/simulation/ScenarioController.hpp"
#include "gui/features/simulation/SimulationController.hpp"
#include "messaging/MessageBus.hpp"
#include "messaging/SimulationMessageClient.hpp"
#include "messaging/SimulationMessages.hpp"

#include <cassert>

namespace {
namespace messaging = application::messaging;

void TestSimulationEventsPublishCommandsAndUpdateChildModel() {
  messaging::MessageBus bus;
  application::SimulationMessageClient client(bus);
  gui::SimulationController controller(client);

  int startCount = 0;
  double requestedHz = 0.0;
  bool maximumSpeed = false;
  auto startSubscription = bus.Subscribe<messaging::SimulationStartCommand>(
      [&startCount](const auto &) { ++startCount; });
  auto rateSubscription = bus.Subscribe<messaging::SimulationRateCommand>(
      [&requestedHz](const auto &command) { requestedHz = command.hz; });
  auto maximumSubscription =
      bus.Subscribe<messaging::SimulationMaximumSpeedCommand>(
          [&maximumSpeed](
              const auto &command) { maximumSpeed = command.enabled; });

  controller.Handle(gui::SimulationStartRequested{});
  controller.Handle(gui::SimulationRateChanged{120.0});
  controller.Handle(gui::MaximumSimulationSpeedChanged{true});
  assert(startCount == 1);
  assert(requestedHz == 120.0);
  assert(maximumSpeed);

  sim::SimulationSnapshot snapshot;
  snapshot.defaultInitialCondition.altitudeFt = 4000.0;
  controller.Synchronize(snapshot);
  controller.Handle({gui::InitialConditionField::AltitudeFt, 5500.0});
  assert(controller.GetInitialConditionModel().pending.altitudeFt == 5500.0);
}

void TestGNCEventsUpdateModelAndPublishCompleteConfig() {
  messaging::MessageBus bus;
  application::SimulationMessageClient client(bus);
  gui::GNCController controller(client);
  sim::SimulationSnapshot snapshot;
  snapshot.primary.available = true;
  snapshot.primaryAutopilot.available = true;
  controller.Synchronize(snapshot);

  controller.Handle(
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

void TestLinearizationEventPublishesCommand() {
  messaging::MessageBus bus;
  application::SimulationMessageClient client(bus);
  gui::LinearizationController controller(client);
  bool automatic = false;
  auto subscription = bus.Subscribe<messaging::LinearizationConfigCommand>(
      [&automatic](const auto &command) {
        automatic = command.automaticUpdatesEnabled;
      });

  controller.Handle(gui::AutomaticLinearizationChanged{true});
  controller.Handle(gui::LinearizationValueTransformChanged{
      gui::LinearizationValueTransform::SignedLog10});

  assert(automatic);
  assert(controller.GetModel().valueTransform
         == gui::LinearizationValueTransform::SignedLog10);
}

void TestScenarioChildUpdatesDraftAndEmitsLaunchIntent() {
  bool launchReceived = false;
  gui::ScenarioController controller({},
      gui::architecture::EventSink<gui::ScenarioLaunchRequested>{
          [&launchReceived](const gui::ScenarioLaunchRequested &event) {
            launchReceived = event.scenario.commandedRollDeg == 14.0;
          }});
  sim::SimulationScenario draft = controller.GetModel().draft;
  draft.commandedRollDeg = 14.0;

  controller.Handle(gui::ScenarioDraftChanged{draft});
  controller.Handle(gui::ScenarioLaunchRequested{controller.GetModel().draft});

  assert(controller.GetModel().draft.commandedRollDeg == 14.0);
  assert(launchReceived);
}
} // namespace

int main() {
  TestSimulationEventsPublishCommandsAndUpdateChildModel();
  TestGNCEventsUpdateModelAndPublishCompleteConfig();
  TestLinearizationEventPublishesCommand();
  TestScenarioChildUpdatesDraftAndEmitsLaunchIntent();
  return 0;
}
