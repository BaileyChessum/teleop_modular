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
// Created by nova on 28/6/25.
//

#include "teleop_core/commands/Command.hpp"

namespace teleop
{

void Command::initialize(
  const CommandDelegate::WeakPtr & context, const std::string & name,
  const std::vector<Event::SharedPtr> & on, const LoggingInterface::SharedPtr & logging,
  const ParameterInterface::SharedPtr & parameters)
{
  context_ = context;
  name_ = name;
  on_ = on;
  logger_ = logging->get_logger();

  // Subscribe to the given events
  const auto logger = get_logger();
  RCLCPP_DEBUG(logger, "Command %s has %lu invocation events:", get_name().c_str(), on_.size());
  for (const auto & event : on) {
    RCLCPP_DEBUG(logger, "  - Subscribing to event %s", event->get_name().c_str());
    event->subscribe(shared_from_this());
  }

  // Do command implementation specific parameterization

  if (const auto context_shared = context_.lock()) {
    on_initialize("commands." + name + ".", parameters, *context_shared);
  }
  else {
    RCLCPP_ERROR(get_logger(), "Failed to access command delegate when initializing \"%s\"", get_name().c_str());
  }
}

void Command::on_event_invoked(const rclcpp::Time & now)
{
  RCLCPP_DEBUG(get_logger(), "Executing command \"%s\"", get_name().c_str());

  if (const auto context = context_.lock()) {
    execute(*context, now);
  }
  else {
    RCLCPP_ERROR(get_logger(), "Failed to access command delegate when initializing \"%s\"", get_name().c_str());
  }
}

}  // namespace teleop
