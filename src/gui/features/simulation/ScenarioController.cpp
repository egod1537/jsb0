#include "gui/features/simulation/ScenarioController.hpp"

#include "sim/scenario/SimulationScenarioSerializer.hpp"

#include <algorithm>
#include <cctype>
#include <system_error>
#include <utility>

namespace gui {
namespace {
constexpr const char *DefaultScenarioFileName = "c172_roll_hold_5deg.yaml";

std::filesystem::path FindDefaultScenarioDirectory() {
  std::error_code error;
  std::filesystem::path directory = std::filesystem::current_path(error);
  if (error) {
    return std::filesystem::path("scenarios");
  }

  while (!directory.empty()) {
    if (std::filesystem::exists(directory / "CMakeLists.txt", error) && !error
        && std::filesystem::exists(directory / "src", error) && !error) {
      return directory / "scenarios";
    }
    const std::filesystem::path parent = directory.parent_path();
    if (parent == directory) {
      break;
    }
    directory = parent;
  }
  return std::filesystem::current_path() / "scenarios";
}

bool HasYamlExtension(const std::filesystem::path &path) {
  std::string extension = path.extension().string();
  std::transform(extension.begin(),
      extension.end(),
      extension.begin(),
      [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
      });
  return extension == ".yaml" || extension == ".yml";
}
} // namespace

ScenarioController::ScenarioController(std::filesystem::path scenarioDirectory,
    architecture::EventSink<ScenarioLaunchRequested> parentEvents)
    : parentEvents_(std::move(parentEvents)) {
  model_.directory = scenarioDirectory.empty() ? FindDefaultScenarioDirectory()
                                               : std::move(scenarioDirectory);
  model_.suggestedFileName = DefaultScenarioFileName;
}

bool ScenarioController::IsDirty() const {
  return model_.draft != model_.cleanScenario;
}

void ScenarioController::Handle(const ScenarioDraftChanged &event) {
  model_.draft = event.draft;
}

void ScenarioController::NewScenario() {
  model_.draft = sim::SimulationScenario{};
  model_.cleanScenario = model_.draft;
  model_.currentFilePath.clear();
  model_.suggestedFileName = "untitled.yaml";
  SetStatus("Created a new scenario. Use Save As to persist it.", false);
}

void ScenarioController::ResetDefaults() {
  model_.draft = sim::SimulationScenario{};
}

bool ScenarioController::Load(const std::filesystem::path &path) {
  const std::filesystem::path resolvedPath = ResolvePath(path);
  sim::SimulationScenario loadedScenario;
  std::string error;
  if (!sim::SimulationScenarioSerializer::Load(resolvedPath,
          loadedScenario,
          error)) {
    SetStatus(std::move(error), true);
    return false;
  }

  loadedScenario.sourceFile = resolvedPath.string();
  model_.draft = std::move(loadedScenario);
  model_.cleanScenario = model_.draft;
  model_.currentFilePath = resolvedPath;
  model_.suggestedFileName = resolvedPath.filename().string();
  SetStatus("Loaded " + resolvedPath.filename().string(), false);
  return true;
}

bool ScenarioController::Save() {
  if (model_.currentFilePath.empty()) {
    SetStatus("No scenario file is connected. Use Save As.", true);
    return false;
  }
  return SaveAs(model_.currentFilePath);
}

bool ScenarioController::SaveAs(const std::filesystem::path &path) {
  const std::filesystem::path resolvedPath = ResolvePath(path);
  std::string error;
  if (!sim::SimulationScenarioSerializer::Save(resolvedPath,
          model_.draft,
          error)) {
    SetStatus(std::move(error), true);
    return false;
  }

  model_.draft.sourceFile = resolvedPath.string();
  model_.cleanScenario = model_.draft;
  model_.currentFilePath = resolvedPath;
  model_.suggestedFileName = resolvedPath.filename().string();
  SetStatus("Saved " + resolvedPath.filename().string(), false);
  return true;
}

bool ScenarioController::ResolveFileName(std::string_view input,
    std::filesystem::path &path) {
  path = std::filesystem::path(input);
  if (path.empty()) {
    SetStatus("Enter a scenario file name.", true);
    return false;
  }
  if (path.has_parent_path()) {
    SetStatus("Scenario File accepts a file name, not a directory path.", true);
    return false;
  }
  if (path.extension().empty()) {
    path.replace_extension(".yaml");
  }
  if (!HasYamlExtension(path)) {
    SetStatus("Scenario file extension must be .yaml or .yml.", true);
    return false;
  }
  return true;
}

void ScenarioController::Handle(const ScenarioLaunchRequested &event) const {
  parentEvents_.Emit(event);
}

std::filesystem::path ScenarioController::ResolvePath(
    const std::filesystem::path &path) const {
  std::filesystem::path resolved =
      path.is_absolute() ? path : model_.directory / path;
  if (resolved.extension().empty()) {
    resolved.replace_extension(".yaml");
  }
  return resolved.lexically_normal();
}

void ScenarioController::SetStatus(std::string message, bool error) {
  model_.statusMessage = std::move(message);
  model_.statusIsError = error;
}
} // namespace gui
