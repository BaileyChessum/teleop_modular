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
// Created by nova on 6/29/25.
//

#ifndef EVENTLISTENER_HPP
#define EVENTLISTENER_HPP
#include <memory>

namespace teleop
{
/**
 * Interface providing a means for events to alert things that depend on them that they have been invoked.
 */
class EventListener
{
public:
  virtual ~EventListener() = default;

  using WeakPtr = std::weak_ptr<EventListener>;
  using SharedPtr = std::shared_ptr<EventListener>;

  virtual void on_event_invoked(const rclcpp::Time & now) = 0;
};

}  // namespace teleop

#endif  // EVENTLISTENER_HPP
