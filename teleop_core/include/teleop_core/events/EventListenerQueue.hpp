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

#ifndef EVENTLISTENERQUEUE_HPP
#define EVENTLISTENERQUEUE_HPP

#include <queue>
#include <rclcpp/time.hpp>

#include "teleop_core/events/EventListener.hpp"

namespace teleop::internal
{
/**
 * Any event listeners to be invoked are added to this queue, and the execution of EventListener::on_event_invoked is
 * deferred until teleop_modular services the queue
 */
class EventListenerQueue
{
public:
  /**
   * Clears the queue, after calling on_event_invoked for elements in the queue
   */
  void service(const rclcpp::Time & now);

  /**
   * Adds a listener to the queue to be invoked in the future when serviced.
   * @param listener A listener to have on_event_invoked called when serviced.
   */
  void enqueue(const EventListener::WeakPtr & listener);

private:
  std::queue<EventListener::WeakPtr> queue_{};
};

}  // namespace teleop::internal

#endif  // EVENTLISTENERQUEUE_HPP
