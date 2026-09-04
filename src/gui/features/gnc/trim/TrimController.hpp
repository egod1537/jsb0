#pragma once

#include "gui/features/gnc/trim/TrimEvents.hpp"

namespace app {
class SimMessageClient;
}

namespace gui {
class TrimController {
public:
  TrimController(app::SimMessageClient &client,
      gnc::TrimRequest &request, bool &resultOpen, bool &residualOpen,
      bool &inProgress);

  void OnEvent(const TrimRequested &event);
  void OnEvent(const TrimRequestValueChanged &event);
  void OnEvent(const TrimExecutionRequested &event);
  void OnEvent(const TrimViewStateChanged &event);

private:
  app::SimMessageClient &client_;
  gnc::TrimRequest &request_;
  bool &resultOpen_;
  bool &residualOpen_;
  bool &inProgress_;
};
} // namespace gui
