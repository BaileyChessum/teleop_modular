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

#include "teleop_core/events/EventManager.hpp"

namespace teleop::internal
{

EventManager::EventManager(InputManager & inputs)
: inputs_(inputs)
  , queue_(std::make_unique<EventListenerQueue>())
  , items_(queue_, inputs)
{
}

void EventManager::update(const rclcpp::Time & now)
{
  for (auto & item : items_) {
    item->update(now);
  }

  queue_->service(now);
}

}  // namespace teleop::internal
