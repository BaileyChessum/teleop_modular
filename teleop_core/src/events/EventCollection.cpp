//
// Created by nova on 6/29/25.
//

#include "teleop_core/events/EventCollection.hpp"
#include "teleop_core/events/ButtonEvent.hpp"

namespace teleop
{

EventCollection::EventCollection(
  std::weak_ptr<internal::EventListenerQueue> listener_queue,
  InputManager & inputs)
: listener_queue_(std::move(listener_queue))
  , factory_{{"/down",
      [&inputs](const std::string & name,
      const std::weak_ptr<internal::EventListenerQueue> & queue) {
        return std::make_shared<internal::ButtonEvent>(name, queue, true, inputs.get_buttons());
      }},
    {"/up",
      [&inputs](const std::string & name,
      const std::weak_ptr<internal::EventListenerQueue> & queue) {
        return std::make_shared<internal::ButtonEvent>(name, queue, false, inputs.get_buttons());
      }}}
{
}

void EventCollection::clean_up()
{
  for (auto it = items_.begin(); it != items_.end(); ) {
    if (it->second.expired()) {
      it = items_.erase(it);
    } else {
      ++it;
    }
  }
}

size_t EventCollection::size() const
{
  size_t count = 0;
  for (const auto & item : items_) {
    if (!item.second.expired()) {
      ++count;
    }
  }
  return count;
}

Event::SharedPtr EventCollection::operator[](const std::string & index)
{
  auto it = items_.find(index);
  Event::SharedPtr ptr;

  if (it != items_.end()) {
    ptr = it->second.lock();
    if (!ptr) {
      items_.erase(it);
    }
  }

  if (!ptr) {
    ptr = create_event_for_name(index);
    items_[index] = ptr;
  }

  return ptr;
}

/// Used with create_event_for_name
bool ends_with(const std::string & str, const std::string & suffix)
{
  return str.size() >= suffix.size() && str.compare(
    str.size() - suffix.size(), suffix.size(),
    suffix) == 0;
}

Event::SharedPtr EventCollection::create_event_for_name(const std::string & name)
{
  // Check if any special suffixes are included
  auto it =
    std::find_if(
    factory_.begin(), factory_.end(), [&](const auto & item) {
      return ends_with(name, item.first);
    });

  if (it != factory_.end()) {
    return it->second(name, listener_queue_);
  }

  // Otherwise create a standard Event that can be invoked with .invoke() externally
  return std::make_shared<Event>(name, listener_queue_);
}

}  // namespace teleop
