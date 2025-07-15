//
// Created by nova on 7/4/25.
//

#include "teleop_modular/commands/SetButtonCommand.hpp"

namespace teleop
{

void SetButtonCommand::on_initialize(const std::string& prefix, const ParameterInterface::SharedPtr& parameters)
{
  Params params{};

  auto name_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  name_descriptor.name = prefix + "name";
  name_descriptor.description = "The name of the button to set the value for.";
  parameters->declare_parameter(name_descriptor.name, rclcpp::ParameterValue(""), name_descriptor);
  if (rclcpp::Parameter name_param; parameters->get_parameter(name_descriptor.name, name_param))
    params.name = name_param.as_string();

  auto value_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  value_descriptor.name = prefix + "value";
  value_descriptor.description = "The value to set the button to when invoked.";
  parameters->declare_parameter(value_descriptor.name, rclcpp::ParameterValue(true), value_descriptor);
  if (rclcpp::Parameter value_param; parameters->get_parameter(value_descriptor.name, value_param))
    params.value = value_param.as_bool();

  params_ = params;
}

void SetButtonCommand::execute(CommandDelegate& context, const rclcpp::Time& now)
{
  context.get_states().get_buttons().set(params_.name, params_.value);
}

}  // namespace teleop

#include <pluginlib/class_list_macros.hpp>

CLASS_LOADER_REGISTER_CLASS(teleop::SetButtonCommand, teleop::Command);
