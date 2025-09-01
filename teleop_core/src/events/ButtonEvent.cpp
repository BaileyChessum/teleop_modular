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

#include <utility>

#include "teleop_core/events/ButtonEvent.hpp"

namespace teleop::internal
{

ButtonEvent::ButtonEvent(
  std::string name, std::weak_ptr<control_mode::internal::EventListenerQueue> listener_queue,
  bool down, control_mode::InputCollection<control_mode::Button> & buttons)
: Event(std::move(name), std::move(listener_queue)), down_(down)
{
  // To get the name of the button to use, we need to remove the suffix beginning with a /, such as "/down" or "/up"
  // from the end of the event name.
  std::size_t pos = get_name().rfind('/');
  if (pos != std::string::npos) {
    std::string button_name = get_name().substr(0, pos);
    button_ = buttons[button_name];
  } else {
    rclcpp::Logger logger = rclcpp::get_logger("event/" + get_name());
    RCLCPP_WARN(
      logger,
      "Could not find the /down or /up suffix in the event name. Using the event name as the button name.");
    button_ = buttons[get_name()];
  }
}

void ButtonEvent::on_update(const rclcpp::Time & now)
{
  if (previous_value_ != *button_) {
    if (down_) {
      if (*button_ > previous_value_) {
        invoke();
      }
    } else {
      if (*button_ < previous_value_) {
        invoke();
      }
    }
  }

  previous_value_ = *button_;
}

}  // namespace teleop::internal
