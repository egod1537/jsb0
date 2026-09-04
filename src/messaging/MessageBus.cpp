#include "messaging/MessageBus.hpp"

namespace app::messaging {
MessageBus::MessageBus() : state_(std::make_shared<State>()) {}
MessageBus::~MessageBus() = default;
} // namespace app::messaging
