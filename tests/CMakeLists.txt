function(add_jsb_test target source)
  add_executable(${target}
    ${source}
  )

  target_compile_features(${target} PRIVATE
    cxx_std_20
  )

  set_target_properties(${target} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
  )

  target_include_directories(${target} PRIVATE
    ${PROJECT_SOURCE_DIR}/src
  )

  if(ARGN)
    target_link_libraries(${target} PRIVATE
      ${ARGN}
    )
  endif()

  add_test(
    NAME ${target}
    COMMAND ${target}
  )
endfunction()

add_jsb_test(math_tests
  MathTests.cpp
)

add_jsb_test(flightui_tests
  FlightUITests.cpp
  flightui
)

add_jsb_test(linearization_value_transform_tests
  LinearizationValueTransformTests.cpp
  jsb_application
)

add_jsb_test(simulation_tests
  SimulationTests.cpp
  jsb_application
)

add_jsb_test(scenario_tests
  ScenarioTests.cpp
  jsb_application
)

add_jsb_test(local_view_state_tests
  LocalViewStateTests.cpp
  jsb_application
)

add_jsb_test(scenario_execution_tests
  ScenarioExecutionTests.cpp
  jsb_application
)

add_jsb_test(telemetry_tests
  TelemetryTests.cpp
  jsb_application
)

add_jsb_test(linearization_perturbation_tests
  LinearizationPerturbationTests.cpp
  jsb_application
)

add_jsb_test(linearization_state_vector_tests
  LinearizationStateVectorTests.cpp
  jsb_application
)

add_jsb_test(dynamic_mode_analyzer_tests
  DynamicModeAnalyzerTests.cpp
  jsb_application
)

add_jsb_test(flight_camera_controller_tests
  FlightCameraControllerTests.cpp
  jsb_application
)

add_jsb_test(flight_path_history_tests
  FlightPathHistoryTests.cpp
  jsb_application
)

add_jsb_test(editor_icon_registry_tests
  EditorIconRegistryTests.cpp
  jsb_application
)

add_jsb_test(editor_layout_manager_tests
  EditorLayoutManagerTests.cpp
  jsb_application
)

add_executable(px4_roll_tuning_probe
  Px4RollTuningProbe.cpp
)

target_compile_features(px4_roll_tuning_probe PRIVATE
  cxx_std_20
)

set_target_properties(px4_roll_tuning_probe PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)

target_include_directories(px4_roll_tuning_probe PRIVATE
  ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(px4_roll_tuning_probe PRIVATE
  jsb_application
)
