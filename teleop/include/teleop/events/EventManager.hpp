//
// Created by nova on 7/9/25.
//

#ifndef TELEOP_EVENTMANAGER_HPP
#define TELEOP_EVENTMANAGER_HPP

#include <memory>
#include "teleop/events/EventListenerQueue.hpp"
#include "teleop/events/EventCollection.hpp"

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

  std::shared_ptr<EventListenerQueue> queue_;
  EventCollection items_;
};

}  // namespace teleop

#endif  // TELEOP_EVENTMANAGER_HPP
