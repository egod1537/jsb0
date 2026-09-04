#include "flightui/latex/LatexRenderer.hpp"

#include <imgui.h>

#include <memory>
#include <utility>

namespace {
class UnavailableLatexRenderer final : public ui::LatexRenderer {
public:
  void Render(const std::string &, const ui::LatexOptions &) override {
    ImGui::TextDisabled("[LaTeX renderer unavailable]");
  }
};

std::unique_ptr<ui::LatexRenderer> &RendererSlot() {
  static std::unique_ptr<ui::LatexRenderer> renderer;
  return renderer;
}
} // namespace

namespace ui::internal {
void SetLatexRenderer(std::unique_ptr<LatexRenderer> renderer) {
  RendererSlot() = std::move(renderer);
}

LatexRenderer &GetLatexRenderer() {
  if (RendererSlot()) {
    return *RendererSlot();
  }

  static UnavailableLatexRenderer unavailableRenderer;
  return unavailableRenderer;
}
} // namespace ui::internal
