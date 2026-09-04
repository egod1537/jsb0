#pragma once

#include "gui/features/gnc/GNCEvents.hpp"
#include "gui/features/gnc/GNCModel.hpp"
#include "gui/features/gnc/experimental/ExperimentalController.hpp"
#include "gui/features/gnc/px4/attitude/Px4AttitudeController.hpp"
#include "gui/features/gnc/px4/tecs/TecsController.hpp"
#include "gui/features/gnc/trim/TrimController.hpp"

namespace app {
class SimMessageClient;
}

namespace gui {
class GNCController {
public:
  explicit GNCController(app::SimMessageClient &client);

  const GNCModel &GetModel() const { return model_; }
  void Synchronize(const sim::SimSnapshot &snapshot);
  void PublishConfiguration(const sim::SimSnapshot &snapshot);

  void OnEvent(const TrimRequested &event);
  void OnEvent(const ManualControlChanged &event);
  void OnEvent(const PrimaryRollHoldConfigChanged &event);
  void OnEvent(const BaselineRollHoldConfigChanged &event);
  void OnEvent(const PrimaryRollHoldValueChanged &event);
  void OnEvent(const BaselineRollHoldValueChanged &event);
  void OnEvent(const BaselineRollHoldTuningResetRequested &event);
  void OnEvent(const BaselinePitchHoldTuningResetRequested &event);
  void OnEvent(const BaselineTecsValueChanged &event);
  void OnEvent(const BaselineTecsParameterChanged &event);
  void OnEvent(const BaselineTecsTuningResetRequested &event);
  void OnEvent(const BaselineTecsAltitudeCaptureRequested &event);
  void OnEvent(const BaselineTecsAirspeedCaptureRequested &event);
  void OnEvent(const TrimRequestValueChanged &event);
  void OnEvent(const TrimExecutionRequested &event);
  void OnEvent(const ExperimentalViewStateChanged &event);
  void OnEvent(const Px4AttitudeViewStateChanged &event);
  void OnEvent(const TrimViewStateChanged &event);

private:
  app::SimMessageClient &client_;
  GNCModel model_;
  ExperimentalController experimentalController_;
  Px4AttitudeController px4AttitudeController_;
  TecsController tecsController_;
  TrimController trimController_;
};
} // namespace gui
