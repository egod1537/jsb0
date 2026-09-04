add_library(jsb_telemetry_contracts STATIC
  telemetry/TelemetryContracts.cpp
  telemetry/TelemetryMetadata.cpp
)
target_compile_features(jsb_telemetry_contracts PUBLIC cxx_std_20)
set_target_properties(jsb_telemetry_contracts PROPERTIES
  CXX_SCAN_FOR_MODULES OFF
)
target_include_directories(jsb_telemetry_contracts
  PUBLIC ${PROJECT_SOURCE_DIR}/src
)

add_library(jsb_sim_core STATIC
  analysis/LinearizationService.cpp
  telemetry/TelemetryChannel.cpp
  telemetry/TelemetryRegistry.cpp
  telemetry/SimTelemetryPublisher.cpp
  telemetry/recording/TelemetryRecordingService.cpp
  Aircraft.cpp
  linearization/AsyncAircraftLinearizer.cpp
  linearization/AircraftLinearizer.cpp
  linearization/DynamicModeAnalyzer.cpp
  linearization/DynamicModeHistory.cpp
  linearization/LinearizationResult.cpp
  linearization/LinearizationPerturbation.cpp
  linearization/LinearizationStateVector.cpp
  ErrorTracker.cpp
  execution/ExecutionVariant.cpp
  execution/ExecutionVariantResolver.cpp
  InitialCondition.cpp
  scenario/SimScenario.cpp
  scenario/SimScenarioSerializer.cpp
  control/ManualFlightControlController.cpp
  control/FlightControlManager.cpp
  gnc/TrimSolver.cpp
  gnc/TrimService.cpp
  gnc/TrimWorkflow.cpp
  gnc/config/Px4ControlProfile.cpp
  gnc/trim/AircraftTrimReference.cpp
  gnc/autopilot/MyAutopilot.cpp
  gnc/autopilot/PX4Autopilot.cpp
  gnc/autopilot/AutopilotFactory.cpp
  gnc/autopilot/experimental/ExperimentalAutopilotFactory.cpp
  gnc/autopilot/px4/Px4AutopilotFactory.cpp
  gnc/control/attitude/Px4RollController.cpp
  gnc/control/attitude/Px4PitchController.cpp
  gnc/control/lateral/Px4CourseController.cpp
  gnc/control/yaw/Px4YawRateController.cpp
  gnc/tecs/Px4TecsController.cpp
  gnc/control/legacy/RollHoldController.cpp
  gnc/control/legacy/PitchHoldController.cpp
  gnc/control/legacy/AirspeedHoldController.cpp
  gnc/control/legacy/YawDamperController.cpp
  gnc/control/legacy/AltitudeHoldController.cpp
  scenario/ScenarioExecutor.cpp
  jsbsim/ControlSystem.cpp
  jsbsim/Properties.cpp
  jsbsim/EngineSystem.cpp
  jsbsim/FDMStateAccess.cpp
  Simulation.cpp
  gnc/filters/WashoutFilter.cpp
)

target_compile_features(jsb_sim_core PUBLIC cxx_std_20)
target_precompile_headers(jsb_sim_core PRIVATE
  <memory>
  <optional>
  <string>
  <utility>
  <vector>
)
set_target_properties(jsb_sim_core PROPERTIES CXX_SCAN_FOR_MODULES OFF)
if(JSB_ENABLE_UNITY_BUILD)
  set_target_properties(jsb_sim_core PROPERTIES
    UNITY_BUILD ON
    UNITY_BUILD_BATCH_SIZE 4
  )
endif()
target_include_directories(jsb_sim_core PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(jsb_sim_core
  PUBLIC
    jsb_common
    jsb_telemetry_contracts
    libJSBSim
    Eigen3::Eigen
  PRIVATE
    jsb_contract
    yaml-cpp::yaml-cpp
)

add_library(jsb_sim_runtime STATIC
  runtime/AutopilotConfigurationService.cpp
  runtime/SimInstanceSet.cpp
  runtime/SimComparison.cpp
  runtime/SimRuntime.cpp
  runtime/SimSnapshotBuilder.cpp
)
target_compile_features(jsb_sim_runtime PUBLIC cxx_std_20)
set_target_properties(jsb_sim_runtime PROPERTIES CXX_SCAN_FOR_MODULES OFF)
target_include_directories(jsb_sim_runtime PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(jsb_sim_runtime PUBLIC jsb_sim_core)

add_executable(jsb_contract_parameter_export
  tools/ParameterContractExporter.cpp
)
target_compile_features(jsb_contract_parameter_export PRIVATE cxx_std_20)
set_target_properties(jsb_contract_parameter_export PROPERTIES
  CXX_SCAN_FOR_MODULES OFF
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)
target_link_libraries(jsb_contract_parameter_export PRIVATE jsb_sim_core)
target_compile_definitions(jsb_contract_parameter_export PRIVATE
  JSB_CONTRACT_VERSION="${JSB_CONTRACT_VERSION}"
)

set(JSB_GENERATED_EXECUTION_CONTRACT
  "${JSB_CONTRACT_GENERATED_ROOT}/execution"
)
add_custom_target(contract_generate_parameters
  COMMAND "$<TARGET_FILE:jsb_contract_parameter_export>"
    --output "${JSB_GENERATED_EXECUTION_CONTRACT}"
  DEPENDS jsb_contract_parameter_export
  COMMENT "Generating execution parameter contract from C++ metadata"
  VERBATIM
)
add_custom_target(contract_parameter_consistency
  COMMAND "$<TARGET_FILE:jsb_contract_parameter_export>"
    --output "${JSB_GENERATED_EXECUTION_CONTRACT}"
  COMMAND ${Python3_EXECUTABLE}
    "${PROJECT_SOURCE_DIR}/scripts/contract_tool.py"
    verify-parameters
    --root "${PROJECT_SOURCE_DIR}"
    --generated "${JSB_GENERATED_EXECUTION_CONTRACT}"
  DEPENDS jsb_contract_parameter_export
  COMMENT "Checking committed parameter contract against C++ metadata"
  VERBATIM
)
add_dependencies(contract_validate contract_parameter_consistency)
add_dependencies(contract_export contract_parameter_consistency)
