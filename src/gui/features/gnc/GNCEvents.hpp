#pragma once

#include "gui/features/gnc/experimental/ExperimentalEvents.hpp"
#include "gui/features/gnc/px4/attitude/Px4AttitudeEvents.hpp"
#include "gui/features/gnc/px4/tecs/TecsEvents.hpp"
#include "gui/features/gnc/trim/TrimEvents.hpp"
#include "sim/control/ControlInput.hpp"

namespace gui {
struct ManualControlChanged {
  control::ControlInput input;
};
} // namespace gui
