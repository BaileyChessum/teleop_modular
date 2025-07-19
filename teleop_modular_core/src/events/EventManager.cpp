//
// Created by nova on 7/9/25.
//

#include "teleop_modular_core/events/EventManager.hpp"

namespace teleop::internal
{

EventManager::EventManager(InputManager& inputs)
  : inputs_(inputs)
  , queue_(std::make_unique<EventListenerQueue>())
  , items_(queue_, inputs)
{
}

void EventManager::update(const rclcpp::Time& now)
{
  for (auto& item : items_) {
    item->update(now);
  }

  queue_->service(now);
}

}  // namespace teleop::internal