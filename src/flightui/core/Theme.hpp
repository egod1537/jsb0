#pragma once

#include <imgui.h>

namespace FlightUI {
enum class SemanticColor {
  Success,
  Warning,
  Error,
};

void ApplyDarkEditorTheme();
ImVec4 GetDarkEditorApplicationBackground();
ImVec4 GetDarkEditorSemanticColor(SemanticColor color);
} // namespace FlightUI
