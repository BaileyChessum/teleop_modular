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
// Created by nova on 1/3/70.
//

#include "teleop_core/events/EventListenerQueue.hpp"
#include <rclcpp/logging.hpp>

namespace teleop::internal
{
void EventListenerQueue::service(const rclcpp::Time & now)
{
  if (queue_.empty()) {
    return;
  }

  while (!queue_.empty()) {
    auto listener_weak_ptr = (queue_.front());
    auto listener = listener_weak_ptr.lock();
    queue_.pop();

    if (listener) {
      listener->on_event_invoked(now);
    }
  }
}

void EventListenerQueue::enqueue(const EventListener::WeakPtr & listener)
{
  queue_.push(listener);
}
}  // namespace teleop::internal
