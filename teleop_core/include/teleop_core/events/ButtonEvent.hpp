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
// Created by nova on 7/9/25.
//

#ifndef TELEOP_MODULAR_BUTTONEVENT_HPP
#define TELEOP_MODULAR_BUTTONEVENT_HPP

#include "Event.hpp"
#include "control_mode/input_ptr.hpp"
#include "control_mode/input_collection.hpp"
#include "teleop_core/inputs/input_pipeline_builder.hpp"

namespace teleop::internal
{

class ButtonEvent : public control_mode::Event, public InputPipelineBuilder::Element
{
public:
  /**
   * Constructor.
   * @param down  When true, the event will be invoked when the button is pressed. When false, the event will be invoked
   * when the event is released.
   */
  ButtonEvent(
    std::string name, std::weak_ptr<control_mode::internal::EventListenerQueue> listener_queue, bool down);

  void link_inputs(const InputManager::Props& previous, InputManager::Props& next, const DeclaredNames& names) override;;

  // TODO: Rename to make clear that these are the inputs we want to consume, not provide
  /**
     * Allows an element to declare what inputs it CONSUMES, not provides. This is useful for any dynamic remapping of
     * any previous elements in the pipeline.
     * \param[in, out] names the set accumulating all declared input names. Add names to declare to this set.
   */
  void declare_input_names(DeclaredNames& names) override;;

  /**
     * Callback ran when hardened inputs are available.
   */
  void on_inputs_available(InputManager::Hardened& inputs) override;;

protected:
  void on_update(const rclcpp::Time & now) override;

private:
  /// The button to listen to
  std::string button_name_;
  Button button_;
  uint8_t previous_value_ = 0;

  /// When true, the event will be invoked when the button is pressed. When false, the event will be invoked when the
  /// event is released.
  const bool down_;
};

}  // namespace teleop

#endif  // TELEOP_MODULAR_BUTTONEVENT_HPP
