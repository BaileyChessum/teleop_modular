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
#include "teleop_core/inputs/Button.hpp"
#include "teleop_core/inputs/InputCollection.hpp"

namespace teleop::internal
{

class ButtonEvent : public Event
{
public:
  /**
   * Constructor.
   * @param down  When true, the event will be invoked when the button is pressed. When false, the event will be invoked
   * when the event is released.
   */
  ButtonEvent(
    std::string name, std::weak_ptr<EventListenerQueue> listener_queue, bool down,
    InputCollection<Button> & buttons);

protected:
  void on_update(const rclcpp::Time & now) override;

private:
  /// The button to listen to
  Button::SharedPtr button_;

  /// When true, the event will be invoked when the button is pressed. When false, the event will be invoked when the
  /// event is released.
  const bool down_;
};

}  // namespace teleop

#endif  // TELEOP_MODULAR_BUTTONEVENT_HPP
