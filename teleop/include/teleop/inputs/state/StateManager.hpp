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
  [[nodiscard]] StateCollection<uint8_t, Button>& get_buttons()
  {
    return buttons_;
  }
  [[nodiscard]] StateCollection<float, Axis>& get_axes()
  {
    return axes_;
  }

private:
  StateCollection<uint8_t, Button> buttons_;
  StateCollection<float, Axis> axes_;
};

}  // namespace teleop

#endif  // STATEMANAGER_HPP
