#include "flightui/controls/Custom.hpp"

#include "flightui/core/UIElementFactory.hpp"

#include <utility>

namespace ui {
UIElement Custom(Action drawAction) {
  return CreateElement(std::move(drawAction));
}
} // namespace ui
