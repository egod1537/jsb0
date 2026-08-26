#pragma once

#include "application/gui/Window.hpp"
#include "application/gui/windows/LinearizationValueTransform.hpp"

#include <string_view>

namespace gnc {
struct LinearizationResult;
}

namespace gui {
class LinearizationWindow final : public Window {
public:
  LinearizationWindow();

protected:
  void OnRender(GUI &gui) override;

private:
  // Display controls
  void DrawTransformSelector();

  // Matrix rendering
  void DrawResult(const gnc::LinearizationResult &result, bool updateInProgress,
      std::string_view errorMessage) const;

  // Display state
  LinearizationValueTransform valueTransform_ =
      LinearizationValueTransform::Raw;
};
} // namespace gui
