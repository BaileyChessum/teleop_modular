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

#ifndef STATEMANAGER_HPP
#define STATEMANAGER_HPP

#include "StateCollection.hpp"
#include "teleop_core/inputs/input_pipeline_builder.hpp"
#include <set>
#include "teleop_core/inputs/InputManager.hpp"

namespace teleop::state
{

/**
 * A class to manage input values that don't originate from an input source. This could be from ROS2, or commands
 */
class StateManager : public InputPipelineBuilder::Element, public InputPipelineElementDelegate
{
public:
  explicit StateManager(InputManager & inputs)
  : buttons_(*this), axes_(*this)
  {
  }

  // Accessors
  [[nodiscard]] StateCollection<uint8_t, Button> & get_buttons()
  {
    return buttons_;
  }
  [[nodiscard]] StateCollection<float, Axis> & get_axes()
  {
    return axes_;
  }

  /**
     * Add inputs to the builder.
     * \param[in] previous The result of the previous InputPipelineBuilder::Element, to use as a basis for populating
     * next.
     * \param[in,out] next The result of this Element. Always stores the previous result from this Element.
   */
  virtual void link_inputs(const InputManager::Props& previous, InputManager::Props& next, const std::set<std::string>& declared_names) {
    next = previous;

    for (auto& [name, state] : buttons_)
      next.button_builder.declare_aggregate(name, state->reference);

    for (auto& [name, state] : axes_)
      next.axis_builder.declare_aggregate(name, state->reference);
  }

  /**
   * Callback ran when hardened inputs are available.
   */
  virtual void on_inputs_available(InputManager::Hardened& inputs) {
    // I don't think we do anything with hardened inputs?
  }

  virtual void relink() override {
    relink_pipeline();
  }

private:
  StateCollection<uint8_t, Button> buttons_;
  StateCollection<float, Axis> axes_;
};

}  // namespace teleop::state

#endif  // STATEMANAGER_HPP
