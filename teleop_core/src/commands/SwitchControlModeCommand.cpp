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

#include "teleop_core/commands/SwitchControlModeCommand.hpp"
#include "teleop_core/control_modes/control_mode_manager.hpp"

namespace teleop
{

void SwitchControlModeCommand::on_initialize(
  const std::string & prefix,
  const ParameterInterface::SharedPtr & parameters)
{
  Params params{};

  auto to_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  to_descriptor.name = prefix + "to";
  to_descriptor.description = "The name of the control mode to activate.";
  parameters->declare_parameter(to_descriptor.name, rclcpp::ParameterValue(""), to_descriptor);
  if (rclcpp::Parameter to_param; parameters->get_parameter(to_descriptor.name, to_param)) {
    params.to = to_param.as_string();
  }

  params_ = params;
}

void SwitchControlModeCommand::execute(CommandDelegate & context, const rclcpp::Time & now)
{
  context.get_control_modes()->set_control_mode(params_.to);
}

}  // namespace teleop

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(teleop::SwitchControlModeCommand, teleop::Command);
