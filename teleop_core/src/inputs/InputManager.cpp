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
}

InputManager::Hardened InputManager::init(const InputManager::Props& props) {
  button_map_ = props.button_builder.construct();
  axis_map_ = props.axis_builder.construct();

  return {button_map_, axis_map_};
}

}  // namespace teleop
