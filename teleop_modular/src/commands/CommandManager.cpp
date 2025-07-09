//
// Created by nova on 6/28/25.
//

#include "../../include/teleop_modular/commands/CommandManager.hpp"
#include "teleop_modular/utilities/get_parameter.hpp"

namespace teleop_modular
{

void CommandManager::create_command(const std::string& name, EventCollection& events)
{
  // TODO: Ensure the name not already defined
  const auto logger = node_->get_logger();

  std::string type;
  std::vector<std::string> invocation_event_names;

  if (!get_type_for_command(name, type, invocation_event_names))
    return;

  // Get the command class from pluginlib
  std::shared_ptr<Command> command_class = nullptr;

  try
  {
    command_class = loader_->createSharedInstance(type);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(logger, "Failed to find command plugin \"%s\" for command \"%s\"!\nwhat(): %s", type.c_str(),
                 name.c_str(), ex.what());
    return;
  }

  std::vector<Event::SharedPtr> stored_events;
  for (const auto& invocation_event_name : invocation_event_names)
  {
    stored_events.emplace_back(events[invocation_event_name]);
  }

  command_class->initialize(context_, name, stored_events, node_->get_node_logging_interface(),
                            node_->get_node_parameters_interface());
  add(name, command_class);
}

void CommandManager::configure(EventCollection& events)
{
  const auto logger = node_->get_logger();

  // Declare and get parameter for control modes to spawn by default
  auto command_definitions_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  command_definitions_descriptor.name = "commands.names";
  command_definitions_descriptor.description =
      "The names of all commands to create, from parameters under "
      "commands.name.* for all names provided. Teleop_Modular won't know that "
      "parameters in the form commands.name.* exist if name is not in this "
      "array.";
  node_->declare_parameter(command_definitions_descriptor.name, rclcpp::ParameterValue(std::vector<std::string>()),
                           command_definitions_descriptor);
  rclcpp::Parameter command_definitions_param;
  node_->get_parameter(command_definitions_descriptor.name, command_definitions_param);

  // Create each control mode according to the given params
  const auto command_names = command_definitions_param.get_type() == rclcpp::PARAMETER_STRING_ARRAY ?
                                 command_definitions_param.as_string_array() :
                                 std::vector<std::string>();

  // Pluginlib for loading commands dynamically
  loader_ = std::make_unique<pluginlib::ClassLoader<Command>>("teleop_modular", "teleop_modular::Command");

  // List available input source plugins
  try
  {
    std::stringstream available_plugins_log{};
    const auto plugins = loader_->getDeclaredClasses();
    for (const auto& plugin : plugins)
    {
      available_plugins_log << "\n\t- " << plugin;
    }

    RCLCPP_DEBUG(logger, "Registered Command plugins:%s", available_plugins_log.str().c_str());
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(logger, "Failed to list command plugins! what(): %s", ex.what());
    return;
  }

  for (const auto& name : command_names)
  {
    create_command(name, events);
  }
}

bool CommandManager::get_type_for_command(const std::string& name, std::string& source_type,
                                          std::vector<std::string>& invocation_event_names) const
{
  // TODO: Check that the parameter hasn't already been defined
  // TODO: Remember that this parameter has already been defined

  auto parameters = node_->get_node_parameters_interface();

  // Declare parameter and get value for source_type
  auto type_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  type_descriptor.name = "commands." + name + ".type";
  type_descriptor.description = "The name of the plugin to load as the class for the \"" + name + "\" command";
  node_->declare_parameter(type_descriptor.name, rclcpp::ParameterType::PARAMETER_STRING, type_descriptor);
  rclcpp::Parameter type_param;
  const auto type_result = node_->get_parameter(type_descriptor.name, type_param);
  if (type_result)
    source_type = type_param.as_string();

  // Declare parameter and get value for invocation_event_names
  const auto on_any = get_parameter<std::vector<std::string>>(parameters, "commands." + name + ".on_any",
                                                              "The name of the Events that should cause the \"" + name +
                                                                  "\" command to execute. This could be the name of a "
                                                                  "button \"button/down\" to execute when pressed "
                                                                  "down, or \"button/up\" when released.");
  const auto on = get_parameter<std::string>(parameters, "commands." + name + ".on",
                                             "The name of the Event that should cause the \"" + name +
                                                 "\" command to execute. This could be the name of a button "
                                                 "\"button/down\" to execute when pressed down, or \"button/up\" when "
                                                 "released.");

  invocation_event_names.clear();
  if (on_any.has_value())
  {
    invocation_event_names = on_any.value();
  }
  if (on.has_value())
  {
    invocation_event_names.emplace_back(on.value());
  }

  return type_result && !invocation_event_names.empty();
}

std::shared_ptr<Command> CommandManager::operator[](const std::string& index)
{
  return items_[index];
}

void CommandManager::add(const std::string& key, const std::shared_ptr<Command>& value)
{
  items_.insert({ key, value });
}

}  // namespace teleop_modular