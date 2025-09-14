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
// Created by Bailey Chessum on 01/09/2025.
//

#ifndef TELEOP_NODE_INPUT_CHANGE_LISTENER_HPP
#define TELEOP_NODE_INPUT_CHANGE_LISTENER_HPP

#include "teleop_core/inputs/input_pipeline_builder.hpp"
#include <vector>
#include <string>
#include "teleop_core/events/EventCollection.hpp"

namespace {
  constexpr float EPSILON = 1e-6;
}

namespace teleop
{

/**
 * The thing that tracks changes to inputs and logs then when log_inputs:=True
 */
class InputChangeListener : public InputPipelineBuilder::Element
{
public:
  InputChangeListener() = default;
  InputChangeListener(rclcpp::Logger logger, EventCollection& events, int significant_figures = 2);

  /**
   * Add inputs to the builder.
   * \param[in] previous The result of the previous InputPipelineBuilder::Element, to use as a basis for populating
   * next.
   * \param[in,out] next The result of this Element. Always stores the previous result from this Element.
   */
  void link_inputs(const InputManager::Props&, InputManager::Props&, const InputPipelineBuilder::DeclaredNames&) override {};

  /**
   * Callback ran when hardened inputs are available.
   */
  void on_inputs_available(InputManager::Hardened& inputs) override;

  /**
   * Checks for changes to inputs and logs any inputs that change.
   */
  void update();

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("log_inputs");
  int significant_figures_ = 2;
  float sig_figures_mult_ = 100.0;
  float epsilon_ = 0.005;

  EventCollection* events_ = nullptr;

  std::vector<std::string> button_names_{};
  std::vector<uint8_t*> buttons_{};
  std::vector<uint8_t> previous_buttons_{};
  int max_button_name_size_ = 8;

  std::vector<std::string> axis_names_{};
  std::vector<float*> axes_{};
  std::vector<float> previous_axes_{};
  int max_axis_name_size_ = 8;
};

}  // namespace teleop

#endif  // TELEOP_NODE_INPUT_CHANGE_LISTENER_HPP
