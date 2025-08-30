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
// Created by Bailey Chessum on 9/6/25.
//

#ifndef TELEOP_MODULAR_INPUTMANAGER_HPP
#define TELEOP_MODULAR_INPUTMANAGER_HPP

#include <rclcpp/node_interfaces/node_logging_interface.hpp>

#include "teleop_core/inputs/Button.hpp"
#include "teleop_core/inputs/Axis.hpp"
#include "teleop_core/inputs/InputDefinition.hpp"
#include "control_mode/input_collection.hpp"
#include "InputMap.hpp"
#include "InputMapBuilder.hpp"
#include <vector>

namespace teleop
{

/**
 * Class responsible for owning the various maps between input name and input object.
 */
class InputManager
{
  using Button = control_mode::Button;
  using Axis = control_mode::Axis;

public:
  /**
   * Everything you need to initialize the input manager
   */
  struct Props {
    InputMapBuilder<uint8_t> button_builder;
    InputMapBuilder<float> axis_builder;
  };

  /**
   * Like props, but with the builders hardened into input maps
   */
  struct Hardened {
    InputMap<uint8_t, Button>& buttons;
    InputMap<float, Axis>& axes;
  };

  InputManager()
  : button_map_()
    , axis_map_()
  {
  }

  // Add move constructor
  InputManager(InputManager && other) noexcept
  : buttons_(std::move(other.button_map_))
    , axes_(std::move(other.axis_map_))
  {
  }

  // Add move assignment
  InputManager & operator=(InputManager && other) noexcept
  {
    if (this != &other) {
      button_map_ = std::move(other.button_map_);
      axis_map_ = std::move(other.axis_map_);
    }
    return *this;
  }

  // Delete copy constructor and assignment
  InputManager(const InputManager &) = delete;
  InputManager & operator=(const InputManager &) = delete;

  // Accessors
  [[nodiscard]] control_mode::InputCollection<Button> & get_buttons()
  {
    return button_map_;
  }
  [[nodiscard]] control_mode::InputCollection<Axis> & get_axes()
  {
    return axis_map_;
  }

  [[nodiscard]] const control_mode::InputCollection<Button> & get_buttons() const
  {
    return button_map_;
  }
  [[nodiscard]] const control_mode::InputCollection<Axis> & get_axes() const
  {
    return axis_map_;
  }

  Hardened init(const Props & props);

  /**
   * @brief polls the current input state and propagates changes:
   *   - Debounces buttons and axes
   *   - Triggers event updates based on input state
   *   - Services all registered event listeners
   */
  void update(const rclcpp::Time & now);

protected:
  /// Stores a string -> T* map, along with additional memory for aggregation
  InputMap<uint8_t, Button> button_map_ = InputMap<uint8_t, Button>(0, {}, {});

  /// Stores a string -> T* map, along with additional memory for aggregation
  InputMap<float, Axis> axis_map_ = InputMap<float, Axis>(0, {}, {});
};

}  // namespace teleop

#endif  // TELEOP_MODULAR_INPUTMANAGER_HPP
