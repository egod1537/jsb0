#pragma once

#include "gui/features/gnc/trim/TrimEvents.hpp"

namespace application {
class SimulationMessageClient;
}

namespace gui {
class TrimController {
public:
  TrimController(application::SimulationMessageClient &client,
      gnc::TrimRequest &request, bool &resultOpen, bool &residualOpen,
      bool &inProgress);

  void Handle(const TrimRequested &event);
  void Handle(const TrimRequestValueChanged &event);
  void Handle(const TrimExecutionRequested &event);
  void Handle(const TrimViewStateChanged &event);

private:
  application::SimulationMessageClient &client_;
  gnc::TrimRequest &request_;
  bool &resultOpen_;
  bool &residualOpen_;
  bool &inProgress_;
};
} // namespace gui
