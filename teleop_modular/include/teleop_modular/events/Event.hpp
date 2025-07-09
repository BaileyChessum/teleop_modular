//
// Created by Bailey Chessum on 4/6/25.
//

#ifndef EVENT_HPP
#define EVENT_HPP

#include <rclcpp/time.hpp>
#include <rclcpp/logging.hpp>
#include <utility>

#include "EventListener.hpp"
#include "EventListenerQueue.hpp"

namespace teleop_modular
{

/**
 * Class to represent an event. If you want to make and expose your own event in your plugin, don't create a child
 * class. Instead, just request an Event::SharedPtr that you can invoke() through the EventCollection.
 */
class Event
{
public:
  using SharedPtr = std::shared_ptr<Event>;

  explicit Event(std::string name, std::weak_ptr<EventListenerQueue> listener_queue);
  virtual ~Event() = default;

  virtual void invoke();

  /**
   * Makes the given listener be notified whenever the event is invoked.
   */
  void subscribe(const EventListener::WeakPtr& listener);

  /**
   * Called to mark the end of the frame, and update the value of is_invoked()
   */
  void update(const rclcpp::Time& now);

  /**
   * Returns true if this event was invoked before update() was last called
   */
  virtual bool is_invoked();

  // Type conversion
  explicit operator bool();

  // Accessors
  [[nodiscard]] const std::string& get_name() const
  {
    return name_;
  }

  using iterator = std::vector<std::weak_ptr<EventListener>>::iterator;
  using const_iterator = std::vector<std::weak_ptr<EventListener>>::const_iterator;

  iterator begin()
  {
    return listeners_.begin();
  }
  [[nodiscard]] const_iterator begin() const
  {
    return listeners_.begin();
  }
  iterator end()
  {
    return listeners_.end();
  }
  [[nodiscard]] const_iterator end() const
  {
    return listeners_.end();
  }

protected:
  virtual void on_update(const rclcpp::Time& now)
  {
  }

private:
  bool previous_invoked_ = false;
  bool invoked_ = false;

  std::string name_ = "uninitialized_event";

  /// Any Commands or other implementations of EventListener that should be notified when an Event is invoked.
  std::vector<EventListener::WeakPtr> listeners_;
  std::weak_ptr<EventListenerQueue> listener_queue_;
};

}  // namespace teleop_modular

#endif  // EVENT_HPP
