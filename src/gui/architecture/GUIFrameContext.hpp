#pragma once

namespace sim {
struct SimSnapshot;
}

namespace gui {
class EditorIconRegistry;

// Immutable application state plus render-only resources for the legacy frame
// shell. Application services are intentionally excluded; feature controllers
// receive those explicitly at construction.
struct GUIFrameContext {
  const sim::SimSnapshot &simulation;
  EditorIconRegistry &icons;
};
} // namespace gui
