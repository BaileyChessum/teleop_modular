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
// Created by nova on 7/4/25.
//

#include "teleop_core/inputs/state/StateManager.hpp"
#include <rclcpp/logging.hpp>
#include "teleop_core/colors.hpp"

namespace teleop::state
{

void StateManager::link_inputs(const InputManager::Props& previous, InputManager::Props& next,
                               const InputPipelineBuilder::DeclaredNames& names)
{
  next = previous;

  auto logger = rclcpp::get_logger("state_manager");
  RCLCPP_DEBUG(logger, "Linking Buttons:");

  for (auto& [name, state] : buttons_) {
    RCLCPP_DEBUG(logger, "  - " C_INPUT "%s" C_RESET, name.c_str());
    next.button_builder.declare_aggregate(name, state->reference);
  }

  RCLCPP_DEBUG(logger, "Linking Axes:");
  for (auto& [name, state] : axes_)
  {
    RCLCPP_DEBUG(logger, "  - " C_INPUT "%s" C_RESET, name.c_str());
    next.axis_builder.declare_aggregate(name, state->reference);
  }

  input_pipeline_established_ = true;
}

}  // namespace teleop::state
