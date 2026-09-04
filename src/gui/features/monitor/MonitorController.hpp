#pragma once

#include "gui/architecture/EventSink.hpp"
#include "gui/features/monitor/MonitorEvents.hpp"
#include "gui/features/monitor/MonitorInput.hpp"

namespace gui {
class MonitorController {
public:
  explicit MonitorController(
      architecture::EventSink<MonitorAutomaticLinearizationChanged>
          parentEvents = {});

  // Passive snapshot input
  void SetInput(MonitorInput input);
  const MonitorInput &GetInput() const { return input_; }

  // Visualization state and rendering
  const MonitorState &GetState() const { return state_; }
  std::vector<MonitorPlotProps> BuildPlotProps() const;

  // Typed local interaction events
  void OnEvent(const MonitorEvent &event);
  void OnEvent(const MonitorLiveChanged &event);
  void OnEvent(const MonitorViewRangeChanged &event);
  void OnEvent(const MonitorVisibleRangeChanged &event);
  void OnEvent(const MonitorCursorMoved &event);
  void OnEvent(const MonitorSelectedRangeChanged &event);
  void OnEvent(const MonitorZoomRequested &event);
  void OnEvent(const MonitorPanRequested &event);
  void OnEvent(const MonitorTelemetryRangeChanged &event);
  void OnEvent(const MonitorStateChanged &event);
  void OnEvent(const MonitorAutomaticLinearizationChanged &event);

private:
  void UpdateLiveRanges();
  void ClampViewRange();
  void ClampVisibleRange();

  MonitorInput input_;
  MonitorState state_;
  architecture::EventSink<MonitorAutomaticLinearizationChanged> parentEvents_;
};
} // namespace gui
