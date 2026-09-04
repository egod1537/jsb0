#include "gui/windows/samples/SamplePlotWindow.hpp"
#include "flightui/FlightUI.hpp"
#include <array>
#include <cmath>

namespace gui {

SamplePlotWindow::SamplePlotWindow() : Window("ImPlot Test") {}

void SamplePlotWindow::OnRender(const sim::SimSnapshot &) {
  constexpr int PointCount = 240;

  std::array<double, PointCount> xs{};
  std::array<double, PointCount> ys{};

  const double time = ui::GetTime();
  for (int index = 0; index < PointCount; ++index) {
    xs[index] = static_cast<double>(index) / 24.0;
    ys[index] = std::sin(xs[index] + time);
  }

  ui::UIElement plot = ui::Plot("Sample Signal")
                           .Height(300.0F)
                           .XAxisLabel("Time")
                           .YAxisLabel("Value")
                           .AddLine("sin(t)",
                               ui::DataView(xs.data(), xs.size()),
                               ui::DataView(ys.data(), ys.size()));

  plot.Render();
}
} // namespace gui
