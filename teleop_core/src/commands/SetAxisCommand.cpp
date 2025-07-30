// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Created by Bailey Chessum on 31/7/25.
//

#include "teleop_core/commands/SetAxisCommand.hpp"

namespace teleop
{

void SetAxisCommand::on_initialize(const std::string & prefix,
                                   const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters)
{
  Params params{};

  auto name_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  name_descriptor.name = prefix + "name";
  name_descriptor.description = "The name of the axis to set the value for.";
  parameters->declare_parameter(name_descriptor.name, rclcpp::ParameterValue(""), name_descriptor);
  if (rclcpp::Parameter name_param; parameters->get_parameter(name_descriptor.name, name_param)) {
    params.name = name_param.as_string();
  }

  auto value_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  value_descriptor.name = prefix + "value";
  value_descriptor.description = "The value to set the axis to when invoked.";
  parameters->declare_parameter(
      value_descriptor.name, rclcpp::ParameterValue(0.0),
      value_descriptor);
  if (rclcpp::Parameter value_param;
      parameters->get_parameter(value_descriptor.name, value_param))
  {
    params.value = static_cast<float>(value_param.as_double());
  }

  params_ = params;
}

void SetAxisCommand::execute(CommandDelegate & context, const rclcpp::Time & now)
{
  context.get_states().get_axes().set(params_.name, params_.value);
}

}  // namespace teleop

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(teleop::SetAxisCommand, teleop::Command);
