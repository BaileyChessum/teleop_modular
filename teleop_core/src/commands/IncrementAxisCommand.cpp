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

#include "teleop_core/commands/IncrementAxisCommand.hpp"
#include "teleop_core/colors.hpp"
#include "teleop_core/utilities/get_parameter.hpp"

namespace teleop
{

void IncrementAxisCommand::on_initialize(
  const std::string & prefix,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters)
{
  Params params{};

  auto name_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  name_descriptor.name = prefix + "name";
  name_descriptor.description = "The name of the axis to increment the value of.";
  parameters->declare_parameter(name_descriptor.name, rclcpp::ParameterValue(""), name_descriptor);
  if (rclcpp::Parameter name_param; parameters->get_parameter(name_descriptor.name, name_param)) {
    params.name = name_param.as_string();
  }

  // The parameters have funny names to keep different command types distinguishable by their parameter names
  auto by_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  by_descriptor.name = prefix + "by";
  by_descriptor.description = "The amount to increment/decrement the axis by to when invoked.";
  parameters->declare_parameter(
    by_descriptor.name, rclcpp::ParameterValue(0.0),
    by_descriptor);
  if (rclcpp::Parameter by_param;
    parameters->get_parameter(by_descriptor.name, by_param))
  {
    params.by = static_cast<float>(by_param.as_double());
  }

  constexpr double infinity = std::numeric_limits<double>::infinity();
  const auto default_until = params.by < 0.0 ? -infinity : infinity;

  auto until_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  until_descriptor.name = prefix + "until";
  until_descriptor.description =
    "The max (or min when 'by' is negative) amount to increment/decrement the axis until when invoked.";
  parameters->declare_parameter(
    until_descriptor.name, rclcpp::ParameterValue(default_until),
    until_descriptor);
  if (rclcpp::Parameter until_param;
    parameters->get_parameter(until_descriptor.name, until_param))
  {
    params.until = static_cast<float>(until_param.as_double());
  }

  params_.log = utils::get_parameter_or_default<bool>(
    parameters, prefix + "log",
    "Whether to log changes to the value.", true);

  params_ = params;
}

void IncrementAxisCommand::execute(CommandDelegate & context, const rclcpp::Time & now)
{
  const auto logger = context.get_node()->get_logger();
  const auto state = context.get_states().get_axes()[params_.name];
  auto axis = context.get_inputs().get_axes()[params_.name];

  if (!state) {
    if (params_.log) {
      RCLCPP_ERROR(
        logger, C_INPUT "  %s\tinvalid shared pointer!\t" C_QUIET "(state)" C_RESET,
        params_.name.c_str());
    }
    return;   //< This should never be possible
  }

  // Do nothing when limit is reached
  if (params_.by < 0.0 && state->value <= params_.until || params_.by >= 0.0 &&
    state->value >= params_.until)
  {
    if (params_.log) {
      RCLCPP_INFO(
        logger, C_INPUT "  %s\t%.2f\t" C_QUIET "(state, limit reached!)" C_RESET,
        params_.name.c_str(), axis->value());
    }
    return;
  }

  // increment state->value, while limiting against params_.until, depending on the sign of params_.by
  if (params_.by < 0.0) {
    state->value = std::max(state->value + params_.by, params_.until);
  } else {
    state->value = std::min(state->value + params_.by, params_.until);
  }

  if (params_.log) {
    RCLCPP_INFO(
      logger, C_INPUT "  %s\t%.2f\t" C_QUIET "(state)" C_RESET, params_.name.c_str(),
      axis->value());
  }
}

}  // namespace teleop

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(teleop::IncrementAxisCommand, teleop::Command);
