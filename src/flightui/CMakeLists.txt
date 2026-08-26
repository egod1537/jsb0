add_library(flightui STATIC
  core/UICommon.cpp
  core/Theme.cpp
  core/UIFont.cpp
  core/UIScale.cpp
  core/UIElement.cpp
  layout/Window.cpp
  layout/Panel.cpp
  layout/FoldOut.cpp
  layout/KeyValueGrid.cpp
  layout/TabGroup.cpp
  layout/VerticalLayout.cpp
  layout/HorizontalLayout.cpp
  controls/Text.cpp
  controls/Heading.cpp
  controls/Latex.cpp
  controls/ValueLabel.cpp
  controls/Button.cpp
  controls/Combo.cpp
  controls/Toggle.cpp
  controls/Input.cpp
  controls/Slider.cpp
  controls/Separator.cpp
  controls/Space.cpp
  controls/Custom.cpp
  latex/LatexRenderer.cpp
  plot/DataView.cpp
  plot/Plot.cpp
)

target_compile_features(flightui PUBLIC
  cxx_std_20
)

set_target_properties(flightui PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)

target_include_directories(flightui PUBLIC
  ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(flightui PUBLIC
  imgui
  implot
)

target_compile_definitions(flightui PRIVATE
  JSB_INTER_FONT_FILE="${PROJECT_SOURCE_DIR}/assets/fonts/Inter-Regular.ttf"
)
