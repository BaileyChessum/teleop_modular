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

namespace teleop
{

void InputManager::update(const rclcpp::Time & now)
{
  button_map_.update();
  axis_map_.update();

  for (const auto& button: buttons_)
    button->debounce(now);
  for (const auto& axis: axes_)
    axis->debounce(now);
}

void InputManager::init(const InputManager::Props& props) {
  keep_alive_buttons_.clear();
  keep_alive_axes_.clear();

  button_map_ = props.button_builder.construct();
  axis_map_ = props.axis_builder.construct();

  for (auto& [name, input] : button_map_) {
    keep_alive_buttons_.emplace_back(buttons_[name]);
    buttons_[name]->add_definition(input);
  }

  for (auto& [name, input] : axis_map_) {
    keep_alive_axes_.emplace_back(axes_[name]);
    axes_[name]->add_definition(input);
  }
}

}  // namespace teleop
