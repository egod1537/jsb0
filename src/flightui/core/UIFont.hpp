#pragma once

#include <filesystem>

namespace ui {
inline constexpr float BaseUIFontSize = 13.0F;
inline constexpr float MinimumUIFontSize = 11.0F;

std::filesystem::path GetPrimaryUIFontPath();
bool LoadPrimaryUIFont();
float CalculateUIFontScale(float uiScale);
} // namespace ui
