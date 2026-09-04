#pragma once

#include "flightui/core/UIElement.hpp"

#include <string>

namespace ui {
struct LatexOptions {
  float Scale = 1.0F;
};

UIElement Latex(std::string source, LatexOptions options = {});
} // namespace ui
