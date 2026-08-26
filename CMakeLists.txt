cmake_minimum_required(VERSION 3.28)

project(
  jsb-flight-console
  LANGUAGES CXX
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(BuildOptions)
include(Dependencies)
include(ImGui)
include(ImPlot)

add_subdirectory(src/flightui)
add_subdirectory(src/application)

enable_testing()
add_subdirectory(tests)
