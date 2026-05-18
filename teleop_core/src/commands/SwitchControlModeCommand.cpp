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
// Created by Bailey Chessum on 6/29/25.
//

#include "teleop_core/commands/SwitchControlModeCommand.hpp"
#include "teleop_core/control_modes/control_mode_manager.hpp"
#include "teleop_core/utilities/get_parameter.hpp"

namespace teleop
{

using namespace teleop::utils;

void SwitchControlModeCommand::on_initialize(
  const std::string & prefix,
  const ParameterInterface::SharedPtr & parameters,
  CommandDelegate & context)
{
  logger_ = rclcpp::get_logger(get_name());
  Params params{};

  params.activate = get_parameter_or_default(
      parameters, prefix + "activate",
      "A list of control mode names to activate on command execution",
      std::vector<std::string>{});

  params.deactivate = get_parameter_or_default(
      parameters, prefix + "deactivate",
      "A list of control mode names to deactivate on command execution",
      std::vector<std::string>{});

  params.to = get_parameter<std::string>(
      parameters, prefix + "to",
      "Deprecated. The name of the control mode to activate.");

  if (params.to.has_value()) {
    RCLCPP_WARN(get_logger(),
                "The '.to' parameter for the switch_control_mode command \"%s\" is deprecated!\nUse '.activate' and "
                "'.deactivate' instead, providing a list of control mode names to activate and deactivate to each.",
                get_name().c_str());
  }

  params_ = params;
}

void SwitchControlModeCommand::execute(CommandDelegate & context, const rclcpp::Time & now)
{
  if (params_.to.has_value()) {
    context.get_control_modes()->set_control_mode(*params_.to);
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Switching control modes...");
  auto result = context.get_control_modes()->switch_control_mode(params_.deactivate, params_.activate);

  if (!result) {
    RCLCPP_WARN(get_logger(), "Failed to switch control modes.");
  }
}

}  // namespace teleop

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(teleop::SwitchControlModeCommand, teleop::Command);
