#pragma once

#include "application/gui/Window.hpp"
#include "application/sim/scenario/SimulationScenario.hpp"

#include <array>
#include <filesystem>
#include <string>

namespace gui {
class ScenarioWindow final : public gui::Window {
public:
  // Lifetime
  explicit ScenarioWindow(std::filesystem::path scenarioDirectory = {});

  // Editable scenario state
  sim::SimulationScenario &GetScenario();
  const sim::SimulationScenario &GetScenario() const;
  void ResetDefaults();

  // Scenario file management
  void NewScenario();
  bool LoadScenarioFile(const std::filesystem::path &path);
  bool SaveScenarioFile();
  bool SaveScenarioFileAs(const std::filesystem::path &path);
  bool IsDirty() const;
  const std::filesystem::path &GetScenarioDirectory() const;
  const std::filesystem::path &GetCurrentFilePath() const;
  const std::string &GetFileStatusMessage() const;

protected:
  // Window configuration and rendering
  void PrepareWindow() override;
  void OnRender(gui::GUI &gui) override;

private:
  // File management
  void DrawFileSection();
  std::filesystem::path ResolveScenarioPath(
      const std::filesystem::path &path) const;
  bool ResolveFileNameInput(std::filesystem::path &path);
  void SetFileNameInput(const std::filesystem::path &path);
  void SetFileStatus(std::string message, bool error);

  // Section rendering
  void DrawScenarioSection();
  void DrawInitialConditionSection();
  void DrawEnvironmentSection();
  void DrawTrimSection();
  void DrawCommandSection();
  void DrawSimulationSection();
  void DrawAcceptanceCriteriaSection();
  void DrawActions(gui::GUI &gui);

  // Scenario configuration
  sim::SimulationScenario scenario_;
  sim::SimulationScenario cleanScenario_;

  // File state
  std::filesystem::path scenarioDirectory_;
  std::filesystem::path currentFilePath_;
  std::array<char, 260> fileNameInput_{};
  std::string fileStatusMessage_;
  bool fileStatusIsError_ = false;

  // Section view state
  bool scenarioSectionOpen_ = true;
  bool initialConditionSectionOpen_ = false;
  bool environmentSectionOpen_ = false;
  bool trimSectionOpen_ = false;
  bool commandSectionOpen_ = true;
  bool simulationSectionOpen_ = false;
  bool acceptanceSectionOpen_ = false;
};
} // namespace gui
