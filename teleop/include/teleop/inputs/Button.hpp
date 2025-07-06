//
// Created by nova on 7/1/25.
//

#ifndef TELEOP_BUTTON_HPP
#define TELEOP_BUTTON_HPP

#include <utility>
#include "Input.hpp"

namespace teleop
{

class Button final : public InputCommon<bool>
{
public:
  using SharedPtr = std::shared_ptr<Button>;
  using WeakPtr = std::weak_ptr<Button>;

  explicit Button(std::string name) : InputCommon<bool>(std::move(name))
  {
  }

  ~Button()
  {
    // Clear event pointers
    on_pressed_down.reset();
    on_pressed.reset();
    on_pressed_up.reset();
  }

  void export_events(teleop::EventCollection& events) override;
  void update_events(const rclcpp::Time& now) override;

  Event::SharedPtr on_pressed;
  Event::SharedPtr on_pressed_down;
  Event::SharedPtr on_pressed_up;
};

}  // namespace teleop

#endif  // TELEOP_BUTTON_HPP
