//
// Created by nova on 6/29/25.
//

#include "teleop_core/commands/LogCommand.hpp"
#include "teleop_core/colors.hpp"

namespace teleop
{

void LogCommand::on_initialize(
  const std::string & prefix,
  const ParameterInterface::SharedPtr & parameters)
{
  Params params{};

  auto message_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  message_descriptor.name = prefix + "message";
  message_descriptor.description = "The text to log.";
  parameters->declare_parameter(
    message_descriptor.name, rclcpp::ParameterValue(""),
    message_descriptor);
  if (rclcpp::Parameter message_param;
    parameters->get_parameter(message_descriptor.name, message_param))
  {
    params.message = message_param.as_string();
  }

  params_ = params;
}

void LogCommand::execute(CommandDelegate & context, const rclcpp::Time & now)
{
  RCLCPP_INFO(get_logger(), "%s" C_RESET, params_.message.c_str());
}

}  // namespace teleop

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(teleop::LogCommand, teleop::Command);
