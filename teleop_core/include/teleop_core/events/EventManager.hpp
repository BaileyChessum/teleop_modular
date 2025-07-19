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

#ifndef TELEOP_MODULAR_EVENTMANAGER_HPP
#define TELEOP_MODULAR_EVENTMANAGER_HPP

#include <memory>
#include "teleop_core/events/EventListenerQueue.hpp"
#include "teleop_core/events/EventCollection.hpp"

namespace teleop::internal
{

class EventManager final
{
public:
  explicit EventManager(InputManager & inputs);

  [[nodiscard]] EventCollection & get_events()
  {
    return items_;
  }

  void update(const rclcpp::Time & now);

private:
  InputManager & inputs_;

  std::shared_ptr<internal::EventListenerQueue> queue_;
  EventCollection items_;
};

}  // namespace teleop::internal

#endif  // TELEOP_MODULAR_EVENTMANAGER_HPP
