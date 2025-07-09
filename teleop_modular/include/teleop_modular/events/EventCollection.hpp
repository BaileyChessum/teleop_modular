//
// Created by nova on 6/29/25.
//

#ifndef TELEOP_MODULAR_EVENTCOLLECTION_H
#define TELEOP_MODULAR_EVENTCOLLECTION_H

#include <utility>
#include <vector>
#include <memory>
#include <map>
#include "teleop_modular/events/Event.hpp"
#include "teleop_modular/utilities/WeakMapIterator.hpp"
#include "teleop_modular/inputs/InputManager.hpp"

namespace teleop_modular
{

/**
 * A container of Events, where events that don't yet exist are created when an attempt is made to retrieve them.
 */
class EventCollection
{
public:
  explicit EventCollection(std::weak_ptr<EventListenerQueue> listener_queue, InputManager& inputs);

  using iterator = WeakMapIterator<Event, false>;
  using const_iterator = WeakMapIterator<Event, true>;

  Event::SharedPtr operator[](const std::string& index);

  iterator begin()
  {
    return iterator(items_.begin(), &items_);
  }
  iterator end()
  {
    return iterator(items_.end(), &items_);
  }
  [[nodiscard]] const_iterator begin() const
  {
    return const_iterator(items_.begin(), &items_);
  }
  [[nodiscard]] const_iterator end() const
  {
    return const_iterator(items_.end(), &items_);
  }
  [[nodiscard]] const_iterator cbegin() const
  {
    return const_iterator(items_.begin(), &items_);
  }
  [[nodiscard]] const_iterator cend() const
  {
    return const_iterator(items_.end(), &items_);
  }

  void clean_up();

  /**
   * Gets the number of non-expired inputs
   */
  [[nodiscard]] size_t size() const;

private:
  /**
   * Parses the given event name in creating an event, and creates the appropriate event based on that name. For
   * example, events ending in "/down" or "/up" will be events for buttons being pressed down or released.
   * @param[in] name    The name of the event to create. This will also inform the type of event created.
   */
  Event::SharedPtr create_event_for_name(const std::string& name);

  std::map<std::string, std::weak_ptr<Event>> items_{};
  std::weak_ptr<EventListenerQueue> listener_queue_;

  using FactoryFunc = std::function<std::shared_ptr<teleop_modular::Event>(const std::string&,
                                                                   const std::weak_ptr<teleop_modular::EventListenerQueue>&)>;

  /// Used to create special subclasses of Event for various different suffixes
  const std::unordered_map<std::string, FactoryFunc> factory_;
};

}  // namespace teleop_modular

#endif  // TELEOP_MODULAR_EVENTCOLLECTION_H
