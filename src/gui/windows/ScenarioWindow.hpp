#pragma once

#include "gui/features/simulation/ScenarioController.hpp"
#include "gui/features/simulation/SimulationController.hpp"
#include "gui/Window.hpp"
#include "sim/scenario/SimulationScenario.hpp"

#include <array>
#include <filesystem>
#include <memory>
#include <string>

namespace gui {
class ScenarioWindow final : public gui::Window {
public:
  // Lifetime
  explicit ScenarioWindow(std::filesystem::path scenarioDirectory = {});
  ScenarioWindow(SimulationController &controller,
      std::filesystem::path scenarioDirectory = {});

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
  void OnRender(const sim::SimulationSnapshot &snapshot) override;

private:
  // File management
  void DrawFileSection();
  bool ResolveFileNameInput(std::filesystem::path &path);
  void SetFileNameInput(const std::filesystem::path &path);

  // Section rendering
  void DrawScenarioSection();
  void DrawInitialConditionSection();
  void DrawEnvironmentSection();
  void DrawTrimSection();
  void DrawCommandSection();
  void DrawSimulationSection();
  void DrawAcceptanceCriteriaSection();
  void DrawActions(const sim::SimulationSnapshot &snapshot);

  // Dependencies
  std::unique_ptr<ScenarioController> ownedController_;
  ScenarioController *controller_ = nullptr;

  // Per-frame editable view copy
  sim::SimulationScenario renderDraft_;

  // File input view state
  std::array<char, 260> fileNameInput_{};

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
