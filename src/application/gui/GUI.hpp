#pragma once

#include "application/SimulationExecutionControl.hpp"
#include "application/gui/Component.hpp"
#include "application/gui/GUIConfig.hpp"
#include "application/gui/Window.hpp"
#include "application/gui/layout/EditorLayoutManager.hpp"
#include "application/gui/layout/EditorWindowStateSettings.hpp"
#include "application/gui/platform/FileDialogService.hpp"
#include "application/gui/resources/EditorIconRegistry.hpp"
#include "application/sim/Simulation.hpp"
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <implot.h>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

namespace gui {
class FlightVizWindow;

class GUI {
public:
  // Lifetime and frame loop
  GUI(sim::Simulation &primarySimulation, sim::Simulation *baselineSimulation,
      GUIConfig config = {});
  ~GUI();

  GUI(const GUI &other) = delete;
  GUI &operator=(const GUI &other) = delete;

  bool Start();
  void Tick();
  void Exit();

  // Window state
  bool ShouldClose() const;
  void RequestClose();

  const GUIConfig &GetConfig() const { return config_; }
  EditorIconRegistry &GetEditorIcons() { return editorIcons_; }
  const EditorIconRegistry &GetEditorIcons() const { return editorIcons_; }
  EditorLayoutManager &GetEditorLayouts() { return editorLayoutManager_; }
  const EditorLayoutManager &GetEditorLayouts() const {
    return editorLayoutManager_;
  }
  IFileDialog &GetFileDialog() { return fileDialogService_; }
  void ResetEditorLayoutToDefault();

  // Application control
  void SetSimulationExecutionControl(
      application::SimulationExecutionControl *control) {
    simulationExecutionControl_ = control;
  }
  application::SimulationExecutionControl &GetSimulationExecutionControl() {
    return *simulationExecutionControl_;
  }
  const application::SimulationExecutionControl &
  GetSimulationExecutionControl() const {
    return *simulationExecutionControl_;
  }

  // Simulation and visualization
  sim::Simulation &GetPrimarySimulation() { return primarySimulation_; }
  const sim::Simulation &GetPrimarySimulation() const {
    return primarySimulation_;
  }
  sim::Simulation *GetBaselineSimulation() { return baselineSimulation_; }
  const sim::Simulation *GetBaselineSimulation() const {
    return baselineSimulation_;
  }

  // UI registration
  void RegisterComponent(std::unique_ptr<Component> component);
  void RegisterWindow(std::unique_ptr<Window> window);

  template <typename T, typename... Args> T &RegisterComponent(Args &&...args) {
    static_assert(std::is_base_of_v<Component, T>,
        "T must inherit from gui::Component");

    auto component = std::make_unique<T>(std::forward<Args>(args)...);
    T &componentRef = *component;
    RegisterComponent(std::move(component));
    return componentRef;
  }

  template <typename T, typename... Args> T &RegisterWindow(Args &&...args) {
    static_assert(std::is_base_of_v<Window, T>,
        "T must inherit from gui::Window");

    auto window = std::make_unique<T>(std::forward<Args>(args)...);
    T &windowRef = *window;
    RegisterWindow(std::move(window));
    return windowRef;
  }

private:
  // Frame lifecycle
  void BeginFrame();
  void RenderFrame();
  void EndFrame();

  // Rendering
  void UpdateUIScale(bool force = false);
  void RenderDockSpace();
  void InitializeDefaultDockLayout(ImGuiID dockSpaceId, ImVec2 dockSpaceSize);
  void RenderMainMenuBar();
  void RenderSimulationMenu();
  void RenderWindowMenu();

  // Component lifecycle
  void StartComponents();
  void TickComponents();

  // Platform state
  GLFWwindow *window_ = nullptr;
  bool initialized_ = false;
  bool glfwInitialized_ = false;
  bool imguiContextCreated_ = false;
  bool glfwBackendInitialized_ = false;
  bool openGlBackendInitialized_ = false;

  // Responsive UI state
  ImGuiStyle baseImGuiStyle_;
  ImPlotStyle baseImPlotStyle_;
  float appliedUIScale_ = 0.0F;

  // UI ownership
  EditorIconRegistry editorIcons_;
  ImGuiEditorLayoutBackend editorLayoutBackend_;
  EditorLayoutManager editorLayoutManager_;
  NativeFileDialogService fileDialogService_;
  std::string workspaceIniPathString_;
  std::vector<std::unique_ptr<Component>> components_;
  std::vector<Window *> windows_;
  EditorWindowStateSettings windowStateSettings_;
  FlightVizWindow *primaryFlightVizWindow_ = nullptr;
  FlightVizWindow *baselineFlightVizWindow_ = nullptr;
  bool defaultDockLayoutInitialized_ = false;

  // Application dependencies
  application::SimulationExecutionControl *simulationExecutionControl_ =
      nullptr;
  sim::Simulation &primarySimulation_;
  sim::Simulation *const baselineSimulation_;

  // Configuration
  GUIConfig config_;
};
} // namespace gui
