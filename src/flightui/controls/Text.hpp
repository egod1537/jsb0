#pragma once

#include "flightui/core/UIElement.hpp"

#include <string>

namespace ui {
UIElement Text(std::string text);
UIElement TextDisabled(std::string text);
UIElement TextWrapped(std::string text);
} // namespace ui
