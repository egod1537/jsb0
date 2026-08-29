#pragma once

#include "gui/architecture/EventSink.hpp"
#include "gui/features/simulation/SimulationEvents.hpp"
#include "sim/scenario/SimulationScenario.hpp"

#include <filesystem>
#include <string>

namespace gui {
struct ScenarioFileModel {
  sim::SimulationScenario draft;
  sim::SimulationScenario cleanScenario;
  std::filesystem::path directory;
  std::filesystem::path currentFilePath;
  std::string suggestedFileName;
  std::string statusMessage;
  bool statusIsError = false;
};

struct ScenarioDraftChanged {
  sim::SimulationScenario draft;
};

class ScenarioController {
public:
  explicit ScenarioController(std::filesystem::path scenarioDirectory = {},
      architecture::EventSink<ScenarioLaunchRequested> parentEvents = {});

  // Immutable file state for the scenario view
  const ScenarioFileModel &GetModel() const { return model_; }
  sim::SimulationScenario &EditDraftForCompatibility() { return model_.draft; }
  bool IsDirty() const;

  // Scenario and file intents
  void Handle(const ScenarioDraftChanged &event);
  void NewScenario();
  void ResetDefaults();
  bool Load(const std::filesystem::path &path);
  bool Save();
  bool SaveAs(const std::filesystem::path &path);
  bool ResolveFileName(std::string_view input, std::filesystem::path &path);
  void Handle(const ScenarioLaunchRequested &event) const;

private:
  std::filesystem::path ResolvePath(const std::filesystem::path &path) const;
  void SetStatus(std::string message, bool error);

  architecture::EventSink<ScenarioLaunchRequested> parentEvents_;
  ScenarioFileModel model_;
};
} // namespace gui
