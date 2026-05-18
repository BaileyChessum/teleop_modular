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
// Created by Bailey Chessum on 4/6/25.
//

#ifndef TELEOP_CORE_EVENT_HPP
#define TELEOP_CORE_EVENT_HPP

#include <rclcpp/time.hpp>
#include <rclcpp/logging.hpp>
#include <utility>

#include "control_mode/event/event.hpp"

namespace teleop
{

using Event = control_mode::Event;

}  // namespace teleop

#endif  // TELEOP_CORE_EVENT_HPP
