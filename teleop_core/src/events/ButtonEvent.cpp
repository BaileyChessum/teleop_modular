//
// Created by nova on 7/9/25.
//

#include <utility>

#include "teleop_core/events/ButtonEvent.hpp"

namespace teleop::internal
{

ButtonEvent::ButtonEvent(
  std::string name, std::weak_ptr<EventListenerQueue> listener_queue,
  bool down, InputCollection<Button> & buttons)
: Event(std::move(name), std::move(listener_queue)), down_(down)
{
  // To get the name of the button to use, we need to remove the suffix beginning with a /, such as "/down" or "/up"
  // from the end of the event name.
  std::size_t pos = get_name().rfind('/');
  if (pos != std::string::npos) {
    std::string button_name = get_name().substr(0, pos);
    button_ = buttons[button_name];
  } else {
    rclcpp::Logger logger = rclcpp::get_logger("event/" + get_name());
    RCLCPP_WARN(
      logger,
      "Could not find the /down or /up suffix in the event name. Using the event name as the button name.");
    button_ = buttons[get_name()];
  }
}

void ButtonEvent::on_update(const rclcpp::Time & now)
{
  if (button_->changed()) {
    if (down_) {
      if (button_->value()) {
        invoke();
      }
    } else {
      if (!button_->value()) {
        invoke();
      }
    }
  }
}

}  // namespace teleop::internal
