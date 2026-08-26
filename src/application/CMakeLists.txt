add_library(jsb_application STATIC
  Application.cpp
  telemetry/TelemetryChannel.cpp
  telemetry/TelemetryRegistry.cpp
  input/Input.cpp
  sim/Aircraft.cpp
  sim/linearizer/AsyncAircraftLinearizer.cpp
  sim/linearizer/AircraftLinearizer.cpp
  sim/linearizer/DynamicModeAnalyzer.cpp
  sim/linearizer/DynamicModeHistory.cpp
  sim/linearizer/LinearizationResult.cpp
  sim/linearizer/LinearizationPerturbation.cpp
  sim/linearizer/LinearizationStateVector.cpp
  sim/Component.cpp
  sim/ErrorTracker.cpp
  sim/InitialCondition.cpp
  sim/StateLogger.cpp
  sim/scenario/SimulationScenario.cpp
  sim/scenario/SimulationScenarioSerializer.cpp
  sim/control/ManualFlightControlController.cpp
  sim/control/FlightControlManager.cpp
  sim/gnc/TrimSolver.cpp
  sim/gnc/TrimService.cpp
  sim/gnc/autopilot/MyAutopilot.cpp
  sim/gnc/autopilot/PX4Autopilot.cpp
  sim/gnc/hold/RollHoldController.cpp
  sim/gnc/hold/Px4RollHoldReferenceController.cpp
  sim/gnc/hold/PitchHoldController.cpp
  sim/gnc/hold/AirspeedHoldController.cpp
  sim/gnc/hold/CourseHoldController.cpp
  sim/gnc/hold/YawDamperController.cpp
  sim/gnc/hold/AltitudeHoldController.cpp
  gui/viz/core/Component.cpp
  gui/viz/core/FlightPathHistory.cpp
  gui/viz/core/GameObject.cpp
  gui/viz/core/Scene.cpp
  gui/viz/render/CameraComponent.cpp
  gui/viz/render/LineCanvas.cpp
  gui/viz/components/GroundGridRenderer.cpp
  gui/viz/components/AltitudeCueRenderer.cpp
  gui/viz/components/FlightCameraController.cpp
  gui/viz/components/AircraftWireframeRenderer.cpp
  gui/viz/components/TelemetryOverlay.cpp
  gui/viz/FlightVisualizer.cpp
  gui/GUI.cpp
  gui/Window.cpp
  gui/resources/EditorIconRegistry.cpp
  gui/layout/EditorLayoutFileSerializer.cpp
  gui/layout/EditorLayoutManager.cpp
  gui/layout/EditorWindowStateSettings.cpp
  gui/layout/Toolbar.cpp
  gui/platform/FileDialogService.cpp
  gui/panels/AutopilotPanel.cpp
  gui/panels/BaselineAutopilotPanel.cpp
  gui/panels/ManualControlPanel.cpp
  gui/panels/TrimPanel.cpp
  gui/windows/SimulationControlWindow.cpp
  gui/windows/SimulationWindow.cpp
  gui/windows/EditorIconBrowserWindow.cpp
  gui/windows/ScenarioWindow.cpp
  gui/windows/FlightConsoleWindow.cpp
  gui/windows/monitor/FlightDataMonitorWindow.cpp
  gui/windows/viz/FlightVizWindow.cpp
  gui/windows/GNCWindow.cpp
  gui/windows/LinearizationValueTransform.cpp
  gui/windows/LinearizationWindow.cpp
  gui/windows/samples/DeclarativeDemoWindow.cpp
  gui/windows/samples/SamplePlotWindow.cpp
  sim/jsbsim/ControlSystem.cpp
  sim/jsbsim/Properties.cpp
  sim/jsbsim/EngineSystem.cpp
  sim/jsbsim/FDMStateAccess.cpp
  flightgear/FlightGearSender.cpp
  flightgear/FlightGearSystem.cpp
  sim/Simulation.cpp
  ${PROJECT_SOURCE_DIR}/src/util/WashoutFilter.cpp
)

target_compile_features(jsb_application PUBLIC
  cxx_std_20
)

target_precompile_headers(jsb_application PRIVATE
  <memory>
  <optional>
  <string>
  <utility>
  <vector>
)

set_target_properties(jsb_application PROPERTIES
  CXX_SCAN_FOR_MODULES OFF
)

if(JSB_ENABLE_UNITY_BUILD)
  set_target_properties(jsb_application PROPERTIES
    UNITY_BUILD ON
    UNITY_BUILD_BATCH_SIZE 4
  )
  set_source_files_properties(
    gui/viz/components/AltitudeCueRenderer.cpp
    gui/resources/EditorIconRegistry.cpp
    PROPERTIES
    SKIP_UNITY_BUILD_INCLUSION ON
  )
endif()

target_include_directories(jsb_application PUBLIC
  ${PROJECT_SOURCE_DIR}/src
)

target_include_directories(jsb_application PRIVATE
  ${stb_SOURCE_DIR}
)

target_compile_definitions(jsb_application PRIVATE
  JSB_UNITY_EDITOR_ICON_DIRECTORY="${PROJECT_SOURCE_DIR}/assets/third_party/unity-editor-icons/icons/small"
)

target_link_libraries(jsb_application PUBLIC
  libJSBSim
  Eigen3::Eigen
  flightui
)

target_link_libraries(jsb_application PRIVATE
  yaml-cpp::yaml-cpp
)

if(WIN32)
  target_link_libraries(jsb_application PUBLIC
    comdlg32
    ws2_32
  )
endif()

add_executable(jsb-flight-console
  ${PROJECT_SOURCE_DIR}/src/main.cpp
)

target_compile_features(jsb-flight-console PRIVATE
  cxx_std_20
)

set_target_properties(jsb-flight-console PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)

target_link_libraries(jsb-flight-console PRIVATE
  jsb_application
)

add_custom_command(
  TARGET jsb-flight-console
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory
          "$<TARGET_FILE_DIR:jsb-flight-console>/assets/fonts"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${PROJECT_SOURCE_DIR}/assets/fonts/Inter-Regular.ttf"
          "$<TARGET_FILE_DIR:jsb-flight-console>/assets/fonts/Inter-Regular.ttf"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${PROJECT_SOURCE_DIR}/assets/fonts/LICENSE.txt"
          "$<TARGET_FILE_DIR:jsb-flight-console>/assets/fonts/LICENSE.txt"
  COMMAND ${CMAKE_COMMAND} -E make_directory
          "$<TARGET_FILE_DIR:jsb-flight-console>/assets/third_party/unity-editor-icons"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          "${PROJECT_SOURCE_DIR}/assets/third_party/unity-editor-icons"
          "$<TARGET_FILE_DIR:jsb-flight-console>/assets/third_party/unity-editor-icons"
  VERBATIM
)
