//
// Created by nova on 7/4/25.
//

#ifndef STATEMANAGER_HPP
#define STATEMANAGER_HPP
#include "StateCollection.hpp"

namespace teleop
{
/**
 * A class to manage input values that don't originate from an input source. This could be from ROS2, or commands
 */
class StateManager
{
public:
  explicit StateManager(InputManager& inputs) : buttons_(inputs.get_buttons()), axes_(inputs.get_axes())
  {
  }

  // Accessors
  [[nodiscard]] StateCollection<bool, Button>& get_buttons()
  {
    return buttons_;
  }
  [[nodiscard]] StateCollection<double, Axis>& get_axes()
  {
    return axes_;
  }

private:
  StateCollection<bool, Button> buttons_;
  StateCollection<double, Axis> axes_;
};

}  // namespace teleop

#endif  // STATEMANAGER_HPP
