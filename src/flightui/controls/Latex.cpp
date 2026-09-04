#include "flightui/controls/Latex.hpp"

#include "flightui/core/UIElementFactory.hpp"
#include "flightui/latex/LatexRenderer.hpp"

#include <utility>

namespace ui {
UIElement Latex(std::string source, LatexOptions options) {
  return CreateElement(
      [source = std::move(source), options] {
        internal::GetLatexRenderer().Render(source, options);
      });
}
} // namespace ui
