//
// Created by nova on 7/9/25.
//

#include "teleop_modular/events/Event.hpp"

namespace teleop_modular
{

Event::Event(std::string name, std::weak_ptr<EventListenerQueue> listener_queue)
    : name_(std::move(name)), listener_queue_(std::move(listener_queue))
{
}

void Event::invoke()
{
  if (const auto listener_queue = listener_queue_.lock(); listener_queue)
  {
    for (const auto& listener : listeners_)
    {
      listener_queue->enqueue(listener);
    }
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger(name_), "Failed to get the listener queue.");
  }

  invoked_ = true;
}

void Event::update(const rclcpp::Time& now)
{
  on_update(now);

  previous_invoked_ = invoked_;
  invoked_ = false;
}

void Event::subscribe(const EventListener::WeakPtr& listener)
{
  listeners_.emplace_back(listener);
}

bool Event::is_invoked()
{
  return previous_invoked_;
}

Event::operator bool()
{
  return is_invoked();
}

} // teleop_modular