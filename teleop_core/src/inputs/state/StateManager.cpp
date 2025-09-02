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

namespace teleop::state
{

void StateManager::link_inputs(const InputManager::Props& previous, InputManager::Props& next,
                               const InputPipelineBuilder::DeclaredNames& names)
{
  next = previous;

  for (auto& [name, state] : buttons_)
    next.button_builder.declare_aggregate(name, state->reference);

  for (auto& [name, state] : axes_)
    next.axis_builder.declare_aggregate(name, state->reference);
}

}  // namespace teleop::state
