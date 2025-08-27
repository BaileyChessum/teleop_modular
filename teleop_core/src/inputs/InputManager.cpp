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

#include "teleop_core/inputs/InputManager.hpp"
#include "teleop_core/inputs/InputMapBuilder.hpp"

namespace teleop
{

void InputManager::update(const rclcpp::Time & now)
{
  for (auto & button : buttons_) {
    button->debounce(now);
  }

  for (auto & axis : axes_) {
    axis->debounce(now);
  }
}

void InputManager::init(std::vector<InputDefinition<uint8_t>> button_definitions, std::vector<InputDefinition<float>> axis_definitions) {
  InputMapBuilder<uint8_t> button_builder{};
  for (auto& definition : button_definitions)
    button_builder.declare_aggregate(definition.name, definition.reference);

  InputMapBuilder<float> axis_builder{};
  for (auto& definition : axis_definitions)
    axis_builder.declare_aggregate(definition.name, definition.reference);





}

}  // namespace teleop
