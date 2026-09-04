#pragma once

#include "gui/architecture/EventSink.hpp"
#include "gui/features/simulation/SimEvents.hpp"
#include "sim/scenario/SimScenario.hpp"
#include "sim/execution/ExecutionRequest.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace gui {
struct ScenarioFileModel {
  sim::SimScenario draft;
  sim::SimScenario cleanScenario;
  sim::ExecutionVariant executionVariant = sim::ExecutionVariant::Primary;
  sim::ScenarioSource source;
  std::filesystem::path directory;
  std::filesystem::path currentFilePath;
  std::vector<std::filesystem::path> availableScenarioFiles;
  std::string suggestedFileName;
  std::string statusMessage;
  bool statusIsError = false;
  bool applyPending = false;
  bool lastApplySucceeded = false;
};

struct ScenarioDraftChanged {
  sim::SimScenario draft;
};

struct ExecutionVariantChanged {
  sim::ExecutionVariant variant = sim::ExecutionVariant::Primary;
};

struct ScenarioApplyCompleted {
  bool succeeded = false;
  std::string error;
};

class ScenarioController {
public:
  explicit ScenarioController(std::filesystem::path scenarioDirectory = {},
      architecture::EventSink<ScenarioLaunchRequested> parentEvents = {});

  // Immutable file state for the scenario view
  const ScenarioFileModel &GetModel() const { return model_; }
  sim::SimScenario &EditDraftForCompatibility() { return model_.draft; }
  bool IsDirty() const;

  // Scenario and file intents
  void OnEvent(const ScenarioDraftChanged &event);
  void OnEvent(const ExecutionVariantChanged &event);
  void NewScenario();
  void ResetDefaults();
  void RefreshAvailableScenarios();
  bool Load(const std::filesystem::path &path);
  bool Save();
  bool SaveAs(const std::filesystem::path &path);
  bool ResolveFileName(std::string_view input, std::filesystem::path &path);
  bool Apply();
  void OnEvent(const ScenarioApplyCompleted &event);

private:
  std::filesystem::path ResolvePath(const std::filesystem::path &path) const;
  void SetStatus(std::string message, bool error);

  architecture::EventSink<ScenarioLaunchRequested> parentEvents_;
  ScenarioFileModel model_;
};
} // namespace gui
