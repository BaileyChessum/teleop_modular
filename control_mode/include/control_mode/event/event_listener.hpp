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
// Created by Bailey Chessum on 29/6/25.
//

#ifndef TELEOP_MODULAR_CONTROL_MODE_EVENTLISTENER_HPP
#define TELEOP_MODULAR_CONTROL_MODE_EVENTLISTENER_HPP

#include <memory>
#include <rclcpp/time.hpp>
#include "control_mode/visibility_control.h"

namespace control_mode
{
/**
 * Interface providing a means for events to alert things that depend on them that they have been invoked.
 */
class CONTROL_MODE_PUBLIC EventListener
{
public:
  virtual ~EventListener() = default;

  using WeakPtr = std::weak_ptr<EventListener>;
  using SharedPtr = std::shared_ptr<EventListener>;

  virtual void on_event_invoked(const rclcpp::Time & now) = 0;
};

}  // namespace control_mode

#endif  // EVENTLISTENER_HPP
