#pragma once

#include "gui/features/linearization/LinearizationEvents.hpp"

namespace app {
class SimMessageClient;
}

namespace gui {
struct LinearizationModel {
  LinearizationValueTransform valueTransform = LinearizationValueTransform::Raw;
};

class LinearizationController {
public:
  explicit LinearizationController(
      app::SimMessageClient &client);

  const LinearizationModel &GetModel() const { return model_; }
  void OnEvent(const AutomaticLinearizationChanged &event);
  void OnEvent(const LinearizationValueTransformChanged &event);

private:
  app::SimMessageClient &client_;
  LinearizationModel model_;
};
} // namespace gui
