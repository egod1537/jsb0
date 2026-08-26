#pragma once

#include "application/gui/Component.hpp"
#include "application/gui/resources/EditorIcon.hpp"
#include "imgui.h"
#include <string>

namespace gui {
class Window : public Component {
public:
  explicit Window(std::string title, std::string iconName = {},
      std::string windowId = {});
  ~Window() override;

  const std::string &GetTitle() const { return title_; }
  const std::string &GetIconName() const { return iconName_; }
  const std::string &GetWindowId() const { return windowId_; }
  const std::string &GetWindowLabel() const { return windowLabel_; }

  bool IsVisible() const { return visible_; }
  void SetVisible(bool visible) { visible_ = visible; }
  bool *GetVisiblePtr() { return &visible_; }

protected:
  void OnTick(GUI &gui) final;

  virtual void PrepareWindow();
  virtual ImGuiWindowFlags GetWindowFlags() const;
  virtual void OnRender(GUI &gui) = 0;

private:
  // Title presentation
  void DrawTitleIcon(const EditorIconHandle &icon) const;

  // Window identity and metadata
  std::string title_;
  std::string windowId_;
  std::string windowLabel_;
  std::string iconTitle_;
  std::string iconName_;

  // Runtime state
  bool visible_ = true;
};
} // namespace gui
