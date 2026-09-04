#include "gui/features/linearization/LinearizationController.hpp"

#include "messaging/SimMessageClient.hpp"

namespace gui {
LinearizationController::LinearizationController(
    app::SimMessageClient &client)
    : client_(client) {}

void LinearizationController::OnEvent(
    const AutomaticLinearizationChanged &event) {
  client_.SetAutomaticLinearizationEnabled(event.enabled);
}

void LinearizationController::OnEvent(
    const LinearizationValueTransformChanged &event) {
  model_.valueTransform = event.transform;
}
} // namespace gui
