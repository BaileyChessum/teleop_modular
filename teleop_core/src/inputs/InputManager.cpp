//
// Created by Bailey Chessum on 6/9/25.
//

#include "teleop_core/inputs/InputManager.hpp"

namespace teleop
{

void InputManager::update(const rclcpp::Time& now)
{
  for (auto& button : buttons_)
  {
    button->debounce(now);
  }

  for (auto& axis : axes_)
  {
    axis->debounce(now);
  }
}

}  // namespace teleop
