//
// Created by nova on 7/9/25.
//

#ifndef TELEOP_MODULAR_EVENTMANAGER_HPP
#define TELEOP_MODULAR_EVENTMANAGER_HPP

#include <memory>
#include "teleop_modular/events/EventListenerQueue.hpp"
#include "teleop_modular/events/EventCollection.hpp"

namespace teleop::internal
{

class EventManager final
{
public:
  explicit EventManager(InputManager& inputs);

  [[nodiscard]] EventCollection& get_events()
  {
    return items_;
  }

  void update(const rclcpp::Time& now);

private:
  InputManager& inputs_;

  std::shared_ptr<internal::EventListenerQueue> queue_;
  EventCollection items_;
};

}  // namespace teleop::internal

#endif  // TELEOP_MODULAR_EVENTMANAGER_HPP
