// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
//
// Created by Bailey Chessum on 6/9/25.
//

#ifndef TELEOP_MODULAR_INPUTMANAGER_HPP
#define TELEOP_MODULAR_INPUTMANAGER_HPP

#include <rclcpp/node_interfaces/node_logging_interface.hpp>

#include "teleop_core/inputs/Button.hpp"
#include "teleop_core/inputs/Axis.hpp"
#include "InputCollection.hpp"

namespace teleop
{

/**
 * Class responsible for owning the various maps between input name and input object.
 */
class InputManager
{
public:
  InputManager()
  : buttons_()
    , axes_()
  {
  }

  // Add move constructor
  InputManager(InputManager && other) noexcept
  : buttons_(std::move(other.buttons_))
    , axes_(std::move(other.axes_))
  {
  }

  // Add move assignment
  InputManager & operator=(InputManager && other) noexcept
  {
    if (this != &other) {
      buttons_ = std::move(other.buttons_);
      axes_ = std::move(other.axes_);
    }
    return *this;
  }

  // Delete copy constructor and assignment
  InputManager(const InputManager &) = delete;
  InputManager & operator=(const InputManager &) = delete;

  // Accessors
  [[nodiscard]] InputCollection<Button> & get_buttons()
  {
    return buttons_;
  }
  [[nodiscard]] InputCollection<Axis> & get_axes()
  {
    return axes_;
  }

  [[nodiscard]] const InputCollection<Button> & get_buttons() const
  {
    return buttons_;
  }
  [[nodiscard]] const InputCollection<Axis> & get_axes() const
  {
    return axes_;
  }

  void init(std::vector<Definition<uint8_t, Button>> button_definitions, std::vector<>);

  /**
   * @brief polls the current input state and propagates changes:
   *   - Debounces buttons and axes
   *   - Triggers event updates based on input state
   *   - Services all registered event listeners
   */
  void update(const rclcpp::Time & now);

protected:
  /**
   * All boolean inputs referenced by an input source or control mode. This collection only holds weak references, and
   * allows Events to be dropped.
   */
  InputCollection<Button> buttons_{};

  /**
   * All double inputs referenced by an input source or control mode. This collection only holds weak references, and
   * allows Events to be dropped.
   */
  InputCollection<Axis> axes_{};
};

}  // namespace teleop

#endif  // TELEOP_MODULAR_INPUTMANAGER_HPP
