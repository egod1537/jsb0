#include "GUI.hpp"
#include "application/gui/windows/EditorIconBrowserWindow.hpp"
#include "application/gui/windows/GNCWindow.hpp"
#include "application/gui/windows/LinearizationWindow.hpp"
#include "application/gui/windows/ScenarioWindow.hpp"
#include "application/gui/windows/SimulationControlWindow.hpp"
#include "application/gui/windows/SimulationWindow.hpp"
#include "application/gui/windows/monitor/FlightDataMonitorWindow.hpp"
#include "application/gui/windows/viz/FlightVizWindow.hpp"
#include "flightui/core/Theme.hpp"
#include "flightui/core/UIFont.hpp"
#include "flightui/core/UIScale.hpp"
#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <utility>

namespace {
constexpr const char *GlslVersion = "#version 130";
constexpr int SwapInterval = 1;
constexpr float UIScaleChangeThreshold = 0.02F;

ImVec2 Scaled(ImVec2 value, float scale) {
  return {value.x * scale, value.y * scale};
}

void ScaleImPlotStyle(ImPlotStyle &style, float scale) {
  style.PlotDefaultSize = Scaled(style.PlotDefaultSize, scale);
  style.PlotMinSize = Scaled(style.PlotMinSize, scale);
  style.PlotBorderSize *= scale;
  style.MajorTickLen = Scaled(style.MajorTickLen, scale);
  style.MinorTickLen = Scaled(style.MinorTickLen, scale);
  style.MajorTickSize = Scaled(style.MajorTickSize, scale);
  style.MinorTickSize = Scaled(style.MinorTickSize, scale);
  style.MajorGridSize = Scaled(style.MajorGridSize, scale);
  style.MinorGridSize = Scaled(style.MinorGridSize, scale);
  style.PlotPadding = Scaled(style.PlotPadding, scale);
  style.LabelPadding = Scaled(style.LabelPadding, scale);
  style.LegendPadding = Scaled(style.LegendPadding, scale);
  style.LegendInnerPadding = Scaled(style.LegendInnerPadding, scale);
  style.LegendSpacing = Scaled(style.LegendSpacing, scale);
  style.MousePosPadding = Scaled(style.MousePosPadding, scale);
  style.AnnotationPadding = Scaled(style.AnnotationPadding, scale);
  style.DigitalPadding *= scale;
  style.DigitalSpacing *= scale;
}
} // namespace

namespace gui {
// public
GUI::GUI(sim::Simulation &primarySimulation,
    sim::Simulation *baselineSimulation, GUIConfig config)
    : editorLayoutManager_(
          EditorLayoutManager::GetDefaultEditorConfigDirectory(),
          &editorLayoutBackend_),
      primarySimulation_(primarySimulation),
      baselineSimulation_(baselineSimulation), config_(std::move(config)) {
  RegisterWindow<SimulationWindow>();
  RegisterWindow<ScenarioWindow>();
  RegisterWindow<GNCWindow>();
  RegisterWindow<LinearizationWindow>();
  RegisterWindow<FlightDataMonitorWindow>();
  primaryFlightVizWindow_ =
      &RegisterWindow<FlightVizWindow>(SimulationSlot::Primary,
          &primarySimulation_,
          baselineSimulation_);
  baselineFlightVizWindow_ =
      &RegisterWindow<FlightVizWindow>(SimulationSlot::Baseline,
          baselineSimulation_,
          &primarySimulation_);
  RegisterWindow<EditorIconBrowserWindow>();
  RegisterComponent<SimulationControlWindow>();
}

GUI::~GUI() { Exit(); }

bool GUI::Start() {
  if (initialized_) {
    return true;
  }

  if (glfwInit() == GLFW_FALSE) {
    std::cerr << "Failed to initialize GLFW\n";
    return false;
  }
  glfwInitialized_ = true;

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  window_ = glfwCreateWindow(config_.windowWidth,
      config_.windowHeight,
      config_.windowTitle.c_str(),
      nullptr,
      nullptr);

  if (window_ == nullptr) {
    std::cerr << "Failed to create GLFW window\n";
    Exit();
    return false;
  }

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(SwapInterval);

  IMGUI_CHECKVERSION();

  ImGui::CreateContext();
  ImPlot::CreateContext();
  imguiContextCreated_ = true;

  ImGuiIO &io = ImGui::GetIO();

  if (!windowStateSettings_.Register(windows_)) {
    std::cerr << "Window visibility settings are unavailable\n";
  }

  if (editorLayoutManager_.Initialize()) {
    workspaceIniPathString_ =
        editorLayoutManager_.GetWorkspaceIniPath().string();
    io.IniFilename = workspaceIniPathString_.c_str();
  } else {
    std::cerr << "Editor layouts are unavailable: "
              << editorLayoutManager_.GetLastError() << '\n';
  }

  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
  // DPI changes affect raster density through the GLFW/OpenGL backends. Keep
  // logical font sizing tied only to the responsive window-resolution scale.
  io.ConfigDpiScaleFonts = false;

  FlightUI::ApplyDarkEditorTheme();
  FlightUI::LoadPrimaryUIFont();
  baseImGuiStyle_ = ImGui::GetStyle();
  baseImPlotStyle_ = ImPlot::GetStyle();
  UpdateUIScale(true);

  if (!ImGui_ImplGlfw_InitForOpenGL(window_, true)) {
    std::cerr << "Failed to initialize ImGui GLFW backend\n";
    Exit();
    return false;
  }
  glfwBackendInitialized_ = true;

  if (!ImGui_ImplOpenGL3_Init(GlslVersion)) {
    std::cerr << "Failed to initialize ImGui OpenGL backend\n";
    Exit();
    return false;
  }
  openGlBackendInitialized_ = true;

  if (!editorIcons_.Initialize()) {
    std::cerr
        << "Editor icons are unavailable; using text-only window titles\n";
  }

  initialized_ = true;
  StartComponents();
  return true;
}

void GUI::Tick() {
  if (!initialized_) {
    return;
  }

  BeginFrame();
  RenderFrame();
  EndFrame();
}

void GUI::Exit() {
  editorIcons_.Shutdown();

  if (openGlBackendInitialized_) {
    ImGui_ImplOpenGL3_Shutdown();
    openGlBackendInitialized_ = false;
  }

  if (glfwBackendInitialized_) {
    ImGui_ImplGlfw_Shutdown();
    glfwBackendInitialized_ = false;
  }

  if (imguiContextCreated_) {
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    imguiContextCreated_ = false;
  }

  initialized_ = false;

  if (window_ != nullptr) {
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }

  if (glfwInitialized_) {
    glfwTerminate();
    glfwInitialized_ = false;
  }
}

void GUI::RegisterComponent(std::unique_ptr<Component> component) {
  if (component == nullptr) {
    return;
  }

  components_.push_back(std::move(component));
  if (initialized_) {
    components_.back()->StartIfNeeded(*this);
  }
}

void GUI::RegisterWindow(std::unique_ptr<Window> window) {
  if (window == nullptr) {
    return;
  }

  windows_.push_back(window.get());
  RegisterComponent(std::move(window));
}

bool GUI::ShouldClose() const {
  return window_ == nullptr || glfwWindowShouldClose(window_);
}

void GUI::RequestClose() {
  if (window_ != nullptr) {
    glfwSetWindowShouldClose(window_, GLFW_TRUE);
  }
}

void GUI::ResetEditorLayoutToDefault() {
  if (!imguiContextCreated_) {
    return;
  }
  ImGui::ClearIniSettings();
  const ImGuiID dockSpaceId = ImGui::GetID("DockSpace");
  ImGui::DockBuilderRemoveNode(dockSpaceId);
  editorLayoutManager_.ClearActivePreset();
  defaultDockLayoutInitialized_ = false;
}

void GUI::BeginFrame() {
  if (!initialized_) {
    return;
  }

  glfwPollEvents();
  UpdateUIScale();
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void GUI::RenderFrame() {
  if (!initialized_) {
    return;
  }

  RenderMainMenuBar();
  RenderDockSpace();
  TickComponents();
}

void GUI::EndFrame() {
  if (!initialized_) {
    return;
  }

  ImGui::Render();

  int displayWidth = 0;
  int displayHeight = 0;
  glfwGetFramebufferSize(window_, &displayWidth, &displayHeight);

  const ImVec4 clearColor = FlightUI::GetDarkEditorApplicationBackground();
  glViewport(0, 0, displayWidth, displayHeight);
  glClearColor(clearColor.x * clearColor.w,
      clearColor.y * clearColor.w,
      clearColor.z * clearColor.w,
      clearColor.w);
  glClear(GL_COLOR_BUFFER_BIT);

  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwSwapBuffers(window_);

  if ((ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) != 0) {
    GLFWwindow *currentContext = glfwGetCurrentContext();
    ImGui::UpdatePlatformWindows();
    ImGui::RenderPlatformWindowsDefault();
    glfwMakeContextCurrent(currentContext);
  }
}

// private
void GUI::UpdateUIScale(bool force) {
  int windowWidth = 0;
  int windowHeight = 0;
  glfwGetWindowSize(window_, &windowWidth, &windowHeight);
  if (windowWidth <= 0 || windowHeight <= 0) {
    return;
  }

  const float uiScale =
      FlightUI::CalculateUIScale(static_cast<float>(windowWidth),
          static_cast<float>(windowHeight));
  if (!force && std::abs(uiScale - appliedUIScale_) < UIScaleChangeThreshold) {
    return;
  }

  appliedUIScale_ = uiScale;
  FlightUI::SetUIScale(uiScale);

  ImGuiStyle scaledImGuiStyle = baseImGuiStyle_;
  scaledImGuiStyle.ScaleAllSizes(uiScale);
  // The current ImGui backend rasterizes dynamically requested font sizes,
  // while framebuffer density remains a separate backend concern.
  scaledImGuiStyle.FontScaleMain =
      baseImGuiStyle_.FontScaleMain * FlightUI::CalculateUIFontScale(uiScale);
  ImGui::GetStyle() = scaledImGuiStyle;

  ImPlotStyle scaledImPlotStyle = baseImPlotStyle_;
  ScaleImPlotStyle(scaledImPlotStyle, uiScale);
  ImPlot::GetStyle() = scaledImPlotStyle;
}

void GUI::RenderDockSpace() {
  const ImGuiViewport *viewport = ImGui::GetMainViewport();
  const float toolbarHeight = SimulationControlWindow::GetReservedHeight();
  const ImVec2 dockSpacePosition{
      viewport->WorkPos.x,
      viewport->WorkPos.y + toolbarHeight,
  };
  const ImVec2 dockSpaceSize{
      viewport->WorkSize.x,
      std::max(viewport->WorkSize.y - toolbarHeight, 1.0F),
  };

  ImGui::SetNextWindowPos(dockSpacePosition);
  ImGui::SetNextWindowSize(dockSpaceSize);
  ImGui::SetNextWindowViewport(viewport->ID);

  constexpr ImGuiWindowFlags HostFlags =
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse
      | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove
      | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoBringToFrontOnFocus
      | ImGuiWindowFlags_NoNavFocus;

  char hostWindowLabel[32]{};
  std::snprintf(hostWindowLabel,
      sizeof(hostWindowLabel),
      "WindowOverViewport_%08X",
      viewport->ID);

  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0F);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
  ImGui::Begin(hostWindowLabel, nullptr, HostFlags);
  ImGui::PopStyleVar(3);

  const ImGuiID dockSpaceId = ImGui::GetID("DockSpace");
  InitializeDefaultDockLayout(dockSpaceId, dockSpaceSize);
  ImGui::DockSpace(dockSpaceId);
  ImGui::End();
}

void GUI::InitializeDefaultDockLayout(ImGuiID dockSpaceId,
    ImVec2 dockSpaceSize) {
  if (defaultDockLayoutInitialized_) {
    return;
  }
  defaultDockLayoutInitialized_ = true;

  const auto dockFlightVizWindows = [this](ImGuiID primaryNode,
                                        ImGuiID baselineNode) {
    ImGui::DockBuilderDockWindow(
        primaryFlightVizWindow_->GetWindowLabel().c_str(),
        primaryNode);
    ImGui::DockBuilderDockWindow(
        baselineFlightVizWindow_->GetWindowLabel().c_str(),
        baselineNode);
  };

  if (ImGui::DockBuilderGetNode(dockSpaceId) != nullptr) {
    const ImGuiID primaryWindowId =
        ImHashStr(primaryFlightVizWindow_->GetWindowLabel().c_str());
    const ImGuiID baselineWindowId =
        ImHashStr(baselineFlightVizWindow_->GetWindowLabel().c_str());
    if (ImGui::FindWindowSettingsByID(primaryWindowId) != nullptr
        || ImGui::FindWindowSettingsByID(baselineWindowId) != nullptr) {
      return;
    }

    ImGuiID targetNode = 0;
    if (const ImGuiWindowSettings *legacySettings =
            ImGui::FindWindowSettingsByID(ImHashStr("Flight Viz"));
        legacySettings != nullptr
        && ImGui::DockBuilderGetNode(legacySettings->DockId) != nullptr) {
      targetNode = legacySettings->DockId;
    } else if (const ImGuiDockNode *centralNode =
                   ImGui::DockBuilderGetCentralNode(dockSpaceId)) {
      targetNode = centralNode->ID;
    }

    if (targetNode != 0) {
      ImGuiID baselineNode = 0;
      ImGuiID primaryNode = 0;
      ImGui::DockBuilderSplitNode(targetNode,
          ImGuiDir_Down,
          0.5F,
          &baselineNode,
          &primaryNode);
      dockFlightVizWindows(primaryNode, baselineNode);
      ImGui::DockBuilderFinish(dockSpaceId);
    }
    return;
  }

  ImGui::DockBuilderRemoveNode(dockSpaceId);
  ImGui::DockBuilderAddNode(dockSpaceId, ImGuiDockNodeFlags_DockSpace);
  ImGui::DockBuilderSetNodeSize(dockSpaceId, dockSpaceSize);

  ImGuiID rightNode = 0;
  ImGuiID leftAndCenterNode = 0;
  ImGui::DockBuilderSplitNode(dockSpaceId,
      ImGuiDir_Right,
      0.30F,
      &rightNode,
      &leftAndCenterNode);
  ImGuiID leftNode = 0;
  ImGuiID centerNode = 0;
  ImGui::DockBuilderSplitNode(leftAndCenterNode,
      ImGuiDir_Left,
      0.34F,
      &leftNode,
      &centerNode);
  ImGuiID baselineNode = 0;
  ImGuiID primaryNode = 0;
  ImGui::DockBuilderSplitNode(centerNode,
      ImGuiDir_Down,
      0.5F,
      &baselineNode,
      &primaryNode);

  dockFlightVizWindows(primaryNode, baselineNode);
  ImGui::DockBuilderDockWindow("GNC", leftNode);
  ImGui::DockBuilderDockWindow("Monitor", leftNode);
  ImGui::DockBuilderDockWindow("FG Linearization", leftNode);
  ImGui::DockBuilderDockWindow("Scenario", rightNode);
  ImGui::DockBuilderDockWindow("Simulation", rightNode);
  ImGui::DockBuilderFinish(dockSpaceId);
}

void GUI::RenderMainMenuBar() {
  if (!ImGui::BeginMainMenuBar()) {
    return;
  }

  RenderSimulationMenu();
  RenderWindowMenu();

  ImGui::EndMainMenuBar();
}

void GUI::RenderSimulationMenu() {
  if (!ImGui::BeginMenu("Simulation")) {
    return;
  }

  auto &executionControl = GetSimulationExecutionControl();
  const application::SimulationExecutionState executionState =
      executionControl.GetSimulationExecutionState();
  const bool scenarioInactive =
      !executionControl.GetScenarioExecutionStatus().has_value();

  ImGui::BeginDisabled(
      executionState != application::SimulationExecutionState::Running);
  if (ImGui::MenuItem("Pause")) {
    executionControl.PauseSimulation();
  }
  ImGui::EndDisabled();

  ImGui::BeginDisabled(
      executionState != application::SimulationExecutionState::Paused);
  if (ImGui::MenuItem("Resume")) {
    executionControl.ResumeSimulation();
  }
  if (ImGui::MenuItem("Tick Once")) {
    executionControl.RequestSimulationTick();
  }
  ImGui::EndDisabled();

  ImGui::BeginDisabled(
      !scenarioInactive
      || executionState == application::SimulationExecutionState::Stopped);
  if (ImGui::MenuItem("Reset")) {
    const bool resumeAfterReset =
        executionState == application::SimulationExecutionState::Running;
    executionControl.PauseSimulation();
    if (executionControl.ResetSimulation() && resumeAfterReset) {
      executionControl.ResumeSimulation();
    }
  }
  ImGui::EndDisabled();

  ImGui::Separator();

  if (ImGui::MenuItem("Exit")) {
    RequestClose();
  }

  ImGui::EndMenu();
}

void GUI::RenderWindowMenu() {
  if (!ImGui::BeginMenu("Window")) {
    return;
  }

  for (Window *window : windows_) {
    if (ImGui::MenuItem(window->GetTitle().c_str(),
            nullptr,
            window->GetVisiblePtr())) {
      ImGui::MarkIniSettingsDirty();
    }
  }

  ImGui::Separator();

  if (ImGui::MenuItem("Show All")) {
    for (Window *window : windows_) {
      window->SetVisible(true);
    }
    ImGui::MarkIniSettingsDirty();
  }

  if (ImGui::MenuItem("Hide All")) {
    for (Window *window : windows_) {
      window->SetVisible(false);
    }
    ImGui::MarkIniSettingsDirty();
  }

  ImGui::EndMenu();
}

void GUI::StartComponents() {
  for (const auto &component : components_) {
    component->StartIfNeeded(*this);
  }
}

void GUI::TickComponents() {
  for (const auto &component : components_) {
    component->Tick(*this);
  }
}
} // namespace gui
