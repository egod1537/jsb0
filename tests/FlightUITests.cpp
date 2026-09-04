#include "flightui/FlightUI.hpp"

#include <cassert>
#include <cmath>
#include <imgui.h>
#include <implot.h>
#include <implot_internal.h>
#include <limits>
#include <string>
#include <utility>
#include <vector>

template <typename T>
concept CanSliderDouble =
    requires { ui::SliderDouble("Value", std::declval<T>(), 0.0, 1.0); };

template <typename T>
concept CanSliderFloat =
    requires { ui::SliderFloat("Value", std::declval<T>(), 0.0F, 1.0F); };

template <typename T>
concept CanSliderInt =
    requires { ui::SliderInt("Value", std::declval<T>(), 0, 10); };

template <typename T>
concept CanToggle = requires { ui::Toggle("Enabled", std::declval<T>()); };

template <typename T>
concept CanMakeDataView = requires { ui::DataView::From(std::declval<T>()); };

template <typename T>
concept CanMakeRingBufferDataView =
    requires(const T &buffer) { buffer.data_view(); };

static_assert(CanSliderDouble<double>);
static_assert(CanSliderDouble<const double &>);
static_assert(CanSliderDouble<double &&>);
static_assert(CanSliderFloat<float>);
static_assert(CanSliderFloat<const float &>);
static_assert(CanSliderFloat<float &&>);
static_assert(CanSliderInt<int>);
static_assert(CanSliderInt<const int &>);
static_assert(CanSliderInt<int &&>);
static_assert(CanToggle<bool>);
static_assert(CanToggle<const bool &>);
static_assert(CanToggle<bool &&>);
static_assert(CanMakeDataView<const std::vector<double> &>);
static_assert(!CanMakeDataView<std::vector<double> &&>);
static_assert(CanMakeDataView<const std::vector<float> &>);
static_assert(!CanMakeDataView<std::vector<float> &&>);
static_assert(CanMakeRingBufferDataView<ds::RingBuffer<double>>);
static_assert(CanMakeRingBufferDataView<ds::RingBuffer<float>>);
static_assert(!CanMakeRingBufferDataView<ds::RingBuffer<int>>);
static_assert(requires {
  ui::Button("Reset")
      .OnAction([] {})
      .Width(80.0F)
      .Height(24.0F)
      .Enabled(true)
      .Tooltip("Reset controls")
      .Id("reset-button");
  ui::Button("Typo Alias").Widht(80.0F);
});
static_assert(requires {
  ui::Toggle("Enabled", true).OnChanged([](bool) {});
  ui::SliderFloat("Value", 0.5F, 0.0F, 1.0F).OnChanged([](float) {});
  ui::SliderDouble("Value", 0.5, 0.0, 1.0).OnChanged([](double) {});
  ui::SliderInt("Value", 5, 0, 10).OnChanged([](int) {});
  ui::SliderDouble("Value", 0.5, 0.0, 1.0).FillAvailableWidth(96.0F);
  ui::ScalarEditor("Gain", 0.5)
      .Range(0.0, 1.0)
      .Step(0.01)
      .FastStep(0.1)
      .Format("%.2f")
      .ShowSlider()
      .ShowInput()
      .ShowStepper()
      .Enabled(true)
      .Tooltip("Gain")
      .OnChanged([](double) {});
  ui::PropertyTable("Properties")
      .LabelWidth(112.0F)
      .ColumnSpacing(4.0F)
      .RowPadding(2.0F)
      .AlternatingRows()
      .Enabled(true)
      .Visible(true)
      .Tooltip("Property values")
      .Add("Gain", ui::Text("0.50"));
  ui::PropertyGrid("Responsive Properties")
      .LabelWidthRatio(0.4F)
      .MinimumLabelWidth(100.0F)
      .MaximumLabelWidth(180.0F)
      .SingleColumnThreshold(320.0F)
      .AlternatingRows()
      .Add(ui::PropertyRow("Gain").Tooltip("Gain value")[ui::Text("0.50")]);
  ui::Toolbar()
      .Id("Tools")
      .Compact()
      .Height(28.0F)
      .Left(ui::Text("Tools"))
      .Right(ui::Button("Action"));
  ui::IconButton("Icon", ImTextureID_Invalid)
      .FallbackText("I")
      .Size(22.0F)
      .Selected()
      .Enabled(true)
      .Tooltip("Icon action")
      .OnAction([] {});
  ui::ToggleIconButton("ToggleIcon", ImTextureID_Invalid, true)
      .OnChanged([](bool) {});
  ui::StatusBadge("Ready", ui::StatusTone::Success);
});
static_assert(requires(bool isOpen) {
  ui::FoldOut("Advanced")
      .Open(isOpen)
      .DefaultOpen()
      .Flags(ImGuiTreeNodeFlags_Framed)
      .HeaderLeft(ui::Toggle("##Selected", true), 18.0F)
      .HeaderRight(ui::Toggle("Enabled", true), 96.0F)
      .Enabled(true)
      .Visible(true)
      .Tooltip("Advanced settings")
      .Id("advanced")[ui::Text("Fold out content")];
  ui::TabGroup("Main Tabs")
      .Flags(ImGuiTabBarFlags_Reorderable)
      .Enabled(true)
      .Visible(true)
      .Tooltip("Main tabs")
      .Id("main-tabs")[+ui::Tab("Controls")
              .Open(isOpen)
              .Flags(ImGuiTabItemFlags_SetSelected)
              .Enabled(true)
              .Visible(true)
              .Tooltip("Controls")
              .Id("controls")[ui::Text("Controls")]];
  ui::ToggleFoldOut("Enabled Section", true)
      .Open(isOpen)
      .DefaultOpen()
      .Section()
      .ToggleEnabled(true)
      .Visible(true)
      .Tooltip("Section")
      .Id("enabled-section")
      .OnChanged([](bool) {})[ui::Text("Content")];
});

int main() {
  constexpr float ScaleTolerance = 0.0001F;
  constexpr double RangeTolerance = 1.0e-9;

  assert(ui::ResolvePropertyGridLayout(319.0F, 320.0F)
         == ui::PropertyGridLayout::SingleColumn);
  assert(ui::ResolvePropertyGridLayout(320.0F, 320.0F)
         == ui::PropertyGridLayout::TwoColumns);
  assert(!ui::IsAlternatePropertyRow(0));
  assert(ui::IsAlternatePropertyRow(1));
  assert(ui::NormalizeScalarEditorValue(0.5, 0.0, 1.0) == 0.5);
  assert(ui::NormalizeScalarEditorValue(-1.0, 0.0, 1.0) == 0.0);
  assert(ui::NormalizeScalarEditorValue(2.0, 0.0, 1.0) == 1.0);
  assert(!ui::NormalizeScalarEditorValue(
              std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0)
              .has_value());

  const auto constantPositiveRange = ui::ExpandYAxisRange(2.0, 2.0);
  assert(constantPositiveRange.has_value());
  assert(std::abs(constantPositiveRange->Min - 1.9) < RangeTolerance);
  assert(std::abs(constantPositiveRange->Max - 2.1) < RangeTolerance);

  const auto constantZeroRange = ui::ExpandYAxisRange(0.0, 0.0);
  assert(constantZeroRange.has_value());
  assert(std::abs(constantZeroRange->Min + 0.1) < RangeTolerance);
  assert(std::abs(constantZeroRange->Max - 0.1) < RangeTolerance);

  const auto constantNegativeRange = ui::ExpandYAxisRange(-3.0, -3.0);
  assert(constantNegativeRange.has_value());
  assert(std::abs(constantNegativeRange->Min + 3.15) < RangeTolerance);
  assert(std::abs(constantNegativeRange->Max + 2.85) < RangeTolerance);

  const auto nearlyConstantRange = ui::ExpandYAxisRange(1.999999, 2.000001);
  assert(nearlyConstantRange.has_value());
  assert(nearlyConstantRange->Min < 1.9);
  assert(nearlyConstantRange->Max > 2.1);

  const auto normalRange = ui::ExpandYAxisRange(-10.0, 10.0);
  assert(normalRange.has_value());
  assert(std::abs(normalRange->Min + 12.0) < RangeTolerance);
  assert(std::abs(normalRange->Max - 12.0) < RangeTolerance);

  const double nanValue = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  assert(!ui::ExpandYAxisRange(nanValue, 1.0).has_value());
  assert(!ui::ExpandYAxisRange(0.0, infinity).has_value());

  assert(
      std::abs(ui::CalculateUIScale(1280.0F, 720.0F) - 1.0F) < ScaleTolerance);
  assert(
      std::abs(ui::CalculateUIScale(1024.0F, 768.0F) - 0.8F) < ScaleTolerance);
  assert(std::abs(ui::CalculateUIScale(640.0F, 360.0F) - ui::MinimumUIScale)
         < ScaleTolerance);
  assert(std::abs(ui::CalculateUIScale(3840.0F, 2160.0F) - ui::MaximumUIScale)
         < ScaleTolerance);

  ui::SetUIScale(ui::CalculateUIScale(1920.0F, 1080.0F));
  assert(std::abs(ui::Ui(100.0F) - 150.0F) < ScaleTolerance);
  ui::SetUIScale(ui::CalculateUIScale(1024.0F, 768.0F));
  assert(std::abs(ui::Ui(100.0F) - 80.0F) < ScaleTolerance);
  const ui::Vector2 scaledPlotSize = ui::UiSize({-1.0F, 245.0F});
  assert(scaledPlotSize.X == -1.0F);
  assert(std::abs(scaledPlotSize.Y - 196.0F) < ScaleTolerance);
  ui::SetUIScale(ui::CalculateUIScale(1920.0F, 1080.0F));
  assert(std::abs(ui::Ui(100.0F) - 150.0F) < ScaleTolerance);
  ui::SetUIScale(1.0F);

  std::vector<double> xValues{0.0, 1.0, 2.0};
  std::vector<double> yValues{0.0, 1.0, 4.0};
  const ui::DataView xView = ui::DataView::From(xValues);
  ds::RingBuffer<double> ringValues(3);

  ringValues.push_back(1.0);
  ringValues.push_back(2.0);
  ringValues.push_back(3.0);
  ringValues.push_back(4.0);

  assert(xView.GetData() == xValues.data());
  assert(xView.GetCount() == xValues.size());
  assert(xView.GetStride() == sizeof(double));
  assert(xView.GetType() == ui::DataType::Double);
  struct StridedPoint {
    double x;
    double y;
  };
  const std::vector<StridedPoint> stridedPoints{{0.0, 1.0}, {2.0, 3.0}};
  const ui::DataView stridedView(&stridedPoints[0].x,
      stridedPoints.size(),
      sizeof(StridedPoint));
  assert(stridedView.GetStride() == sizeof(StridedPoint));
  assert(ringValues.capacity() == 3);
  assert(ringValues.size() == 3);
  assert(ringValues.offset() == 1);
  assert(ringValues[0] == 2.0);
  assert(ringValues[1] == 3.0);
  assert(ringValues[2] == 4.0);
  assert(ringValues.to_vector() == std::vector<double>({2.0, 3.0, 4.0}));

  const ui::DataView ringView = ringValues.data_view();
  assert(ringView.GetData() == ringValues.data());
  assert(ringView.GetCount() == ringValues.size());
  assert(ringView.GetType() == ui::DataType::Double);

  ui::UIElement text = ui::Text(std::string("Temporary text"));
  assert(text.IsValid());
  ui::UIElement temporaryLatex =
      ui::Latex(std::string(R"(\dot{x} = Ax + Bu)"));
  assert(temporaryLatex.IsValid());
  ui::UIElement scaledLatex = ui::Latex(
      R"(\frac{\partial f}{\partial x})", {.Scale = 1.25F});
  assert(scaledLatex.IsValid());

  ImGui::CreateContext();
  ImPlot::CreateContext();
  ui::ApplyDarkEditorTheme();
  assert(ui::LoadPrimaryUIFont());
  assert(ui::GetPrimaryUIFontPath().filename() == "Inter-Regular.ttf");

  const ImGuiStyle &darkEditorStyle = ImGui::GetStyle();
  const ImPlotStyle &darkEditorPlotStyle = ImPlot::GetStyle();
  assert(std::abs(darkEditorStyle.Colors[ImGuiCol_WindowBg].x - 30.0F / 255.0F)
         < ScaleTolerance);
  assert(
      std::abs(darkEditorStyle.Colors[ImGuiCol_CheckMark].z - 255.0F / 255.0F)
      < ScaleTolerance);
  assert(darkEditorStyle.WindowRounding == 5.0F);
  assert(darkEditorStyle.FrameRounding == 4.0F);
  assert(darkEditorStyle.FrameBorderSize == 0.0F);
  assert(darkEditorStyle.WindowPadding.x == 10.0F);
  assert(darkEditorStyle.FramePadding.y == 5.0F);
  assert(darkEditorStyle.Colors[ImGuiCol_Button].x
         < darkEditorStyle.Colors[ImGuiCol_ButtonHovered].x);
  assert(darkEditorStyle.Colors[ImGuiCol_TabSelected].x
         > darkEditorStyle.Colors[ImGuiCol_Tab].x);
  const ImVec4 propertyRowBackground =
      ui::GetThemeColor(ui::ThemeColor::PropertyRowBackground);
  const ImVec4 propertyRowBackgroundAlternate =
      ui::GetThemeColor(ui::ThemeColor::PropertyRowBackgroundAlternate);
  assert(propertyRowBackground.w == 0.0F);
  assert(propertyRowBackgroundAlternate.w > propertyRowBackground.w);
  assert(propertyRowBackgroundAlternate.w < 0.3F);
  const ui::StatusBadgeStyle neutralBadge =
      ui::GetStatusBadgeStyle(ui::StatusTone::Neutral);
  const ui::StatusBadgeStyle successBadge =
      ui::GetStatusBadgeStyle(ui::StatusTone::Success);
  const ui::StatusBadgeStyle errorBadge =
      ui::GetStatusBadgeStyle(ui::StatusTone::Error);
  assert(neutralBadge.Background.w > 0.0F);
  assert(successBadge.Text.y > successBadge.Text.x);
  assert(errorBadge.Text.x > errorBadge.Text.y);
  assert(darkEditorPlotStyle.Colors[ImPlotCol_PlotBg].x
         < darkEditorPlotStyle.Colors[ImPlotCol_FrameBg].x);
  assert(darkEditorPlotStyle.Colors[ImPlotCol_AxisGrid].w < 0.25F);
  assert(ImGui::GetIO().Fonts->Fonts.Size == 1);
  assert(ImGui::GetIO().FontDefault != nullptr);
  assert(std::abs(ImGui::GetIO().FontDefault->LegacySize - ui::BaseUIFontSize)
         < ScaleTolerance);
  assert(std::abs(ui::CalculateUIFontScale(1.0F) - 1.0F) < ScaleTolerance);
  assert(std::abs(ui::CalculateUIFontScale(0.7F) * ui::BaseUIFontSize
                  - ui::MinimumUIFontSize)
         < ScaleTolerance);
  const float largeFontScale = ui::CalculateUIFontScale(1.5F);
  const float smallFontScale = ui::CalculateUIFontScale(0.8F);
  const float restoredFontScale = ui::CalculateUIFontScale(1.5F);
  assert(std::abs(smallFontScale * ui::BaseUIFontSize - ui::MinimumUIFontSize)
         < ScaleTolerance);
  assert(std::abs(restoredFontScale - largeFontScale) < ScaleTolerance);
  assert(std::abs(ui::CalculateUIFontScale(1.5F) - 1.5F) < ScaleTolerance);

  ImGuiIO &io = ImGui::GetIO();
  io.DisplaySize = ImVec2(800.0F, 600.0F);
  io.DeltaTime = 1.0F / 60.0F;
  unsigned char *fontPixels = nullptr;
  int fontWidth = 0;
  int fontHeight = 0;
  io.Fonts->GetTexDataAsRGBA32(&fontPixels, &fontWidth, &fontHeight);
  ImFontBaked *interRegular = io.FontDefault->GetFontBaked(ui::BaseUIFontSize);
  assert(interRegular->FindGlyphNoFallback('A') != nullptr);
  assert(interRegular->FindGlyphNoFallback('0') != nullptr);
  assert(interRegular->FindGlyphNoFallback('%') != nullptr);
  assert(interRegular->FindGlyphNoFallback('/') != nullptr);
  assert(interRegular->FindGlyphNoFallback(0x03B1) != nullptr);
  assert(interRegular->FindGlyphNoFallback(0x03B2) != nullptr);
  assert(interRegular->FindGlyphNoFallback(0x03C6) != nullptr);
  assert(interRegular->FindGlyphNoFallback(0x03B8) != nullptr);
  assert(interRegular->FindGlyphNoFallback(0x03C8) != nullptr);
  assert(interRegular->FindGlyphNoFallback(0x0307) != nullptr);

  bool isOpen = true;
  bool isTabOpen = true;
  bool isFoldOutOpen = true;
  bool enabled = false;
  double throttle = 0.5;
  double linkedXAxisMin = 0.0;
  double linkedXAxisMax = 2.0;
  int clicks = 0;
  std::vector<int> plotCallbackOrder;
  const ImVec4 explicitLineColor(0.15F, 0.35F, 0.75F, 1.0F);
  bool explicitLineColorApplied = false;
  bool plotLegendDisabled = false;

  ImGui::NewFrame();

  ImGui::Begin("LaTeX FlightUI Test");
  temporaryLatex.Render();
  scaledLatex.Render();
  ImGui::End();

  ui::Window("FlightUI Test")
      .Open(isOpen)
      .InitialSize({640.0F, 480.0F})[ui::VerticalLayout({
          ui::Heading("Controls"),
          ui::Text(std::string("Temporary text")),
          ui::FoldOut("Fold Out")
              .Open(isFoldOutOpen)
              .DefaultOpen()
              .Flags(ImGuiTreeNodeFlags_Framed)[ui::Text("Fold out body")],
          ui::Panel("Panel").Border(true)[ui::VerticalLayout({
              ui::Toggle("Enabled", enabled).OnChanged([&enabled](bool value) {
                enabled = value;
              }),
              ui::SliderDouble("Throttle", throttle, 0.0, 1.0)
                  .OnChanged([&throttle](double value) { throttle = value; })
                  .Width(180.0F),
              ui::ValueLabel("Throttle readout", throttle + 0.125, "{:.2f}"),
              ui::Button("Reset")
                  .OnAction([&clicks] { ++clicks; })
                  .Width(80.0F),
              ui::PropertyTable("Test Properties")
                  .LabelWidth(112.0F)
                  .AlternatingRows()
                  .Add("Throttle",
                      ui::SliderDouble("##PropertyThrottle", throttle, 0.0, 1.0)
                          .FillAvailableWidth()),
              ui::StatusBadge("Ready", ui::StatusTone::Success),
              ui::Custom([] { ImGui::TextUnformatted("Custom"); }),
          })],
          ui::TabGroup("Telemetry Tabs")
              .Flags(ImGuiTabBarFlags_Reorderable)
                  [+ui::Tab("Controls")[ui::Text("Control tab")]
                      + ui::Tab("Monitor").Open(isTabOpen).Tooltip(
                          "Monitor tab")[ui::Text("Monitor tab")]],
          ui::Plot("Plot")
              .Offset(1)
              .Height(120.0F)
              .XAxisFlags(ImPlotAxisFlags_None)
              .YAxisFlags(ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit)
              .XAxisLinks(linkedXAxisMin, linkedXAxisMax)
              .XAxisTicks({0.0, 1.0, 2.0})
              .YAxisLimits(0.0, 4.0)
              .XAxisLabel("X")
              .YAxisLabel("Y")
              .AddLine("Line", xValues, yValues)
              .AddLine("Offset Line", xValues, yValues, 2)
              .AddLine("Ring Line", ringView, ringView, ringValues.offset())
              .AddScatter("Scatter", xValues, yValues, 1),
      })];

  // clang-format off
  ui::Window("Slate Style FlightUI Test")
  [
    +ui::Heading("Controls")
    + ui::Panel("Panel")
          .Border(true)
          [
            +ui::Toggle("Enabled", enabled)
                 .OnChanged([&enabled](bool value) { enabled = value; })
            + ui::SliderDouble("Throttle", throttle, 0.0, 1.0)
                  .OnChanged([&throttle](double value) { throttle = value; })
                  .Format("%.2f")
            + ui::Button("Reset").OnAction([&clicks] { ++clicks; })
          ]
    + ui::HorizontalLayout()
          .Spacing(8.0F)
          [
            +ui::Text("Left")
            + ui::Text("Right")
          ]
  ];
  // clang-format on

  ui::UIElement chainedLayout =
      ui::VerticalLayout() + ui::Text("First") + ui::Text("Second");
  assert(chainedLayout.IsValid());

  ui::Window("Second FlightUI Test")[ui::Text("Second window")];

  ImGui::Begin("Plot Callback Order Test");
  ui::UIElement callbackPlot =
      ui::Plot("Callback Order")
          .Height(120.0F)
          .LegendVisible(false)
          .Underlay([&plotCallbackOrder] { plotCallbackOrder.push_back(1); })
          .AddLine("Callback Line", xValues, yValues)
          .AddLine("Explicit Color Line",
              xView,
              ui::DataView::From(yValues),
              explicitLineColor)
          .Overlay([&] {
            plotCallbackOrder.push_back(2);
            const ImPlotItem *item = ImPlot::GetItem("Explicit Color Line");
            explicitLineColorApplied =
                item != nullptr
                && item->Color
                       == ImGui::ColorConvertFloat4ToU32(explicitLineColor);
            const ImPlotPlot *plot = ImPlot::GetCurrentPlot();
            plotLegendDisabled =
                plot != nullptr && (plot->Flags & ImPlotFlags_NoLegend) != 0;
          });
  callbackPlot.Render();
  ImGui::End();

  ImGui::Render();

  assert(isOpen);
  assert(isTabOpen);
  assert(isFoldOutOpen);
  assert(clicks == 0);
  assert(plotCallbackOrder == std::vector<int>({1, 2}));
  assert(explicitLineColorApplied);
  assert(plotLegendDisabled);
  assert(std::abs(linkedXAxisMin) < RangeTolerance);
  assert(std::abs(linkedXAxisMax - 2.0) < RangeTolerance);

  bool controllerOpen = true;
  bool controllerEnabled = false;
  ImVec2 controllerHeaderMinimum{};
  const auto renderControllerHeaderFrame = [&] {
    ImGui::NewFrame();
    ImGui::SetNextWindowPos(ImVec2(20.0F, 20.0F), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(420.0F, 180.0F), ImGuiCond_Always);
    ImGui::Begin("FoldOut Header Interaction Test",
        nullptr,
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse);
    controllerHeaderMinimum = ImGui::GetCursorScreenPos();
    ui::ToggleFoldOut("Roll Hold", controllerEnabled)
        .Open(controllerOpen)
        .DefaultOpen()
        .Id("ControllerHeaderInteraction")
        .OnChanged([&controllerEnabled](bool enabledValue) {
          controllerEnabled = enabledValue;
        })[ui::Text("Controller settings")]
        .Render();
    ImGui::End();
    ImGui::Render();
  };

  io.AddMousePosEvent(-1000.0F, -1000.0F);
  renderControllerHeaderFrame();
  const ImVec2 toggleCenter{controllerHeaderMinimum.x
          + ImGui::GetTreeNodeToLabelSpacing()
          + ImGui::GetFrameHeight() * 0.5F,
      controllerHeaderMinimum.y + ImGui::GetFrameHeight() * 0.5F};
  io.AddMousePosEvent(toggleCenter.x, toggleCenter.y);
  renderControllerHeaderFrame();
  io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
  renderControllerHeaderFrame();
  io.AddMouseButtonEvent(ImGuiMouseButton_Left, false);
  renderControllerHeaderFrame();
  assert(controllerEnabled);
  assert(controllerOpen);

  const ImVec2 titlePosition{
      controllerHeaderMinimum.x + 48.0F, toggleCenter.y};
  io.AddMousePosEvent(titlePosition.x, titlePosition.y);
  renderControllerHeaderFrame();
  io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
  renderControllerHeaderFrame();
  io.AddMouseButtonEvent(ImGuiMouseButton_Left, false);
  renderControllerHeaderFrame();
  assert(controllerEnabled);
  assert(!controllerOpen);
  io.AddMousePosEvent(-1000.0F, -1000.0F);

  ImVec2 toolbarButtonMinimum{};
  ImVec2 toolbarButtonMaximum{};
  const auto renderToolbarFrame = [&] {
    ImGui::NewFrame();
    ImGui::SetNextWindowPos(ImVec2(20.0F, 20.0F), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(420.0F, 120.0F), ImGuiCond_Always);
    ImGui::Begin("Toolbar Alignment Test",
        nullptr,
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse);
    ui::Toolbar()
        .Id("Alignment")
        .AlignRight()
        .Height(28.0F)[ui::Custom([&] {
          ImGui::Button("Action", ImVec2(54.0F, 0.0F));
          toolbarButtonMinimum = ImGui::GetItemRectMin();
          toolbarButtonMaximum = ImGui::GetItemRectMax();
        })]
        .Render();
    ImGui::End();
    ImGui::Render();
  };
  renderToolbarFrame();
  renderToolbarFrame();
  assert(toolbarButtonMinimum.x > 350.0F);
  assert(toolbarButtonMaximum.x < 440.0F);

  const std::vector<double> focusedNearValues{0.0, 1.0, 0.5};
  const std::vector<double> focusedFarValues{100.0, 200.0, 150.0};
  ImPlotRange focusedYAxisRange;
  const auto renderFocusedYAxisFrame = [&] {
    ImGui::NewFrame();
    ImGui::Begin("Focused Y Axis Test");
    ui::UIElement focusedPlot =
        ui::Plot("Hidden Series Range")
            .Height(120.0F)
            .FocusedYAxis()
            .XAxisLimitsAlways(0.0, 2.0)
            .AddLine("Near", xValues, focusedNearValues)
            .AddLine("Far", xValues, focusedFarValues)
            .Overlay([&focusedYAxisRange] {
              if (ImPlotItem *farItem = ImPlot::GetItem("Far")) {
                farItem->Show = false;
              }
              focusedYAxisRange = ImPlot::GetPlotLimits().Y;
            });
    focusedPlot.Render();
    ImGui::End();
    ImGui::Render();
  };

  renderFocusedYAxisFrame();
  renderFocusedYAxisFrame();
  renderFocusedYAxisFrame();
  assert(focusedYAxisRange.Min < 0.0);
  assert(focusedYAxisRange.Max > 1.0);
  assert(focusedYAxisRange.Max < 10.0);

  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  return 0;
}
