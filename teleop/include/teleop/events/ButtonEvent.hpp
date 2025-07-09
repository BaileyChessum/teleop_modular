//
// Created by nova on 7/9/25.
//

#ifndef TELEOP_BUTTONEVENT_HPP
#define TELEOP_BUTTONEVENT_HPP

#include "Event.hpp"
#include "teleop/inputs/Button.hpp"
#include "teleop/inputs/InputCollection.hpp"

namespace teleop::internal
{

class ButtonEvent : public Event
{
public:
  /**
   * Constructor.
   * @param down  When true, the event will be invoked when the button is pressed. When false, the event will be invoked
   * when the event is released.
   */
  ButtonEvent(std::string name, std::weak_ptr<EventListenerQueue> listener_queue, bool down, InputCollection<Button>& buttons);

protected:
  void on_update(const rclcpp::Time &now) override;

private:
  /// The button to listen to
  Button::SharedPtr button_;

  /// When true, the event will be invoked when the button is pressed. When false, the event will be invoked when the
  /// event is released.
  const bool down_;
};

}  // namespace teleop

#endif  // TELEOP_BUTTONEVENT_HPP
