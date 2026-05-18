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
// Created by Bailey Chessum.
//

#ifndef TELEOP_MODULAR_CONTROL_MODE_EVENTLISTENERQUEUE_HPP
#define TELEOP_MODULAR_CONTROL_MODE_EVENTLISTENERQUEUE_HPP

#include <queue>
#include <rclcpp/time.hpp>
#include "control_mode/visibility_control.h"
#include "event_listener.hpp"

namespace control_mode::internal
{
/**
 * Any event listeners to be invoked are added to this queue, and the execution of EventListener::on_event_invoked is
 * deferred until teleop_modular services the queue
 */
class CONTROL_MODE_PUBLIC EventListenerQueue
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

}  // namespace control_mode::internal

#endif  // TELEOP_MODULAR_CONTROL_MODE_EVENTLISTENERQUEUE_HPP
