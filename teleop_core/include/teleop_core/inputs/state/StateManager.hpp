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
  explicit StateManager()
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
  void link_inputs(const InputManager::Props& previous, InputManager::Props& next,
                   const InputPipelineBuilder::DeclaredNames& names) override;

  /**
   * Callback ran when hardened inputs are available.
   */
  void on_inputs_available(InputManager::Hardened& inputs) override {
    // I don't think we do anything with hardened inputs?
  }

  virtual void relink() override {
    if (input_pipeline_established_)
      relink_pipeline();
  }

private:
  bool input_pipeline_established_ = false;

  StateCollection<uint8_t, Button> buttons_;
  StateCollection<float, Axis> axes_;
};

}  // namespace teleop::state

#endif  // STATEMANAGER_HPP
