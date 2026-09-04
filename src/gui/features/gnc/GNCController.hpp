#pragma once

#include "gui/features/gnc/GNCEvents.hpp"
#include "gui/features/gnc/GNCModel.hpp"
#include "gui/features/gnc/experimental/ExperimentalController.hpp"
#include "gui/features/gnc/px4/attitude/Px4AttitudeController.hpp"
#include "gui/features/gnc/px4/tecs/TecsController.hpp"
#include "gui/features/gnc/trim/TrimController.hpp"

namespace application {
class SimulationMessageClient;
}

namespace gui {
class GNCController {
public:
  explicit GNCController(application::SimulationMessageClient &client);

  const GNCModel &GetModel() const { return model_; }
  void Synchronize(const sim::SimulationSnapshot &snapshot);
  void PublishConfiguration(const sim::SimulationSnapshot &snapshot);

  void Handle(const TrimRequested &event);
  void Handle(const ManualControlChanged &event);
  void Handle(const PrimaryRollHoldConfigChanged &event);
  void Handle(const BaselineRollHoldConfigChanged &event);
  void Handle(const PrimaryRollHoldValueChanged &event);
  void Handle(const BaselineRollHoldValueChanged &event);
  void Handle(const BaselineRollHoldTuningResetRequested &event);
  void Handle(const BaselinePitchHoldTuningResetRequested &event);
  void Handle(const BaselineTecsValueChanged &event);
  void Handle(const BaselineTecsParameterChanged &event);
  void Handle(const BaselineTecsTuningResetRequested &event);
  void Handle(const BaselineTecsAltitudeCaptureRequested &event);
  void Handle(const BaselineTecsAirspeedCaptureRequested &event);
  void Handle(const TrimRequestValueChanged &event);
  void Handle(const TrimExecutionRequested &event);
  void Handle(const ExperimentalViewStateChanged &event);
  void Handle(const Px4AttitudeViewStateChanged &event);
  void Handle(const TrimViewStateChanged &event);

private:
  application::SimulationMessageClient &client_;
  GNCModel model_;
  ExperimentalController experimentalController_;
  Px4AttitudeController px4AttitudeController_;
  TecsController tecsController_;
  TrimController trimController_;
};
} // namespace gui
