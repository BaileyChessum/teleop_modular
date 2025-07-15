
#include "teleop_modular/control_modes/ControlModeManager.hpp"

#include <controller_manager_msgs/srv/detail/switch_controller__struct.hpp>
#include "teleop_modular/colors.hpp"
#include "teleop_modular/utils.hpp"

namespace teleop::internal
{

namespace {
  using control_mode::ControlMode;
}

void ControlModeManager::configure(InputManager& inputs)
{
  const auto logger = node_->get_logger();

  // Create service clients
  switch_controller_client_ =
      node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

  auto names_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  names_descriptor.name = "control_modes.names";
  names_descriptor.description =
      "The names of all control modes to create, from parameters under control_modes.name.* "
      "for all names provided. Teleop_Modular won't know that parameters in the form "
      "control_modes.name.* exist if name is not in this array.";

  // Declare and get parameter for control modes to spawn by default
  node_->declare_parameter(names_descriptor.name, rclcpp::ParameterType::PARAMETER_STRING_ARRAY, names_descriptor);
  ;
  rclcpp::Parameter control_modes_param;
  node_->get_parameter(names_descriptor.name, control_modes_param);

  if (control_modes_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    RCLCPP_ERROR(logger, "control_modes.names was not set.");
  }
  else if (control_modes_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
  {
    RCLCPP_ERROR(logger, "control_modes.names parameter must be a string array, but a %s was given.",
                 control_modes_param.get_type_name().c_str());
  }

  const auto control_mode_names = control_modes_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY ?
                                      control_modes_param.as_string_array() :
                                      std::vector<std::string>();

  // Pluginlib for loading control modes dynamically
  control_mode_loader_ = std::make_unique<pluginlib::ClassLoader<ControlMode>>("teleop_modular", "teleop_modular::ControlMode");

  // List available control mode plugins
  try
  {
    std::stringstream available_plugins_log{};
    const auto plugins = control_mode_loader_->getDeclaredClasses();
    for (const auto& plugin : plugins)
    {
      available_plugins_log << "\n\t- " << plugin;
    }

    RCLCPP_DEBUG(logger, "Registered ControlMode plugins:%s", available_plugins_log.str().c_str());
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(logger, "Failed to list control mode plugins! what(): %s", ex.what());
    return;
  }

  // Create each control mode according to the given params
  std::stringstream registered_modes_log{};
  for (auto& control_mode_name : control_mode_names)
  {
    std::string control_mode_type;
    const std::string pretty_name = snake_to_title(control_mode_name);

    // Get the control mode's plugin type name
    if (!get_type_for_control_mode(control_mode_name, control_mode_type))
    {
      RCLCPP_ERROR(logger,
                   "Failed to find type for control mode \"%s\" in params. Have you defined %s.type in your "
                   "parameter file?",
                   control_mode_name.c_str(), control_mode_name.c_str());
      registered_modes_log << C_FAIL_QUIET << "\n\t- " << pretty_name << C_FAIL_QUIET << "\t(failed - "
                           << control_mode_name << ".type param missing) " << C_RESET;
      continue;
    }

    // Get the control mode class from pluginlib
    std::shared_ptr<ControlMode> control_mode_class = nullptr;
    try
    {
      control_mode_class = control_mode_loader_->createSharedInstance(control_mode_type);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(logger, "Failed to find control mode plugin \"%s\" for mode \"%s\"!\nwhat(): %s",
                   control_mode_type.c_str(), control_mode_name.c_str(), ex.what());
      registered_modes_log << C_FAIL_QUIET << "\n\t- " << pretty_name << C_FAIL_QUIET
                           << "\t(failed - can't find plugin " << control_mode_type << ") " << C_RESET;
      continue;
    }

    // Create a node for the control mode
    const auto options =
        rclcpp::NodeOptions(node_->get_node_options()).context(node_->get_node_base_interface()->get_context());

    // Initialize the control mode
    control_mode_class->init(control_mode_name, node_->get_namespace(), options);

    registered_modes_log << "\n\t- " << pretty_name << C_QUIET << "\t: " << control_mode_type << C_RESET;
    control_modes_[control_mode_name] = control_mode_class;
  }

  RCLCPP_INFO(logger, C_TITLE "Control Modes:" C_RESET "%s\n", registered_modes_log.str().c_str());

  auto executor = executor_.lock();

  // Configure each control mode
  for (const auto& [name, control_mode] : control_modes_)
  {
    if (!control_mode)
      continue;
    executor->add_node(control_mode->get_node()->get_node_base_interface());

    control_mode->get_node()->configure();
    control_mode->capture_inputs(inputs);
  }

  executor.reset();
}

void ControlModeManager::activate_initial_control_mode()
{  // Activate the first control mode in the list
  if (!control_modes_.empty())
    set_control_mode(control_modes_.begin()->first);
}

bool ControlModeManager::set_control_mode(const std::string& name)
{
  const auto logger = node_->get_logger();

  // Find the control mode from the name
  const auto new_control_mode_it = std::find_if(
      control_modes_.begin(), control_modes_.end(),
      [name](const std::pair<std::string, std::shared_ptr<ControlMode>>& pair) { return pair.first == name; });

  // Ensure the given control mode exists
  if (new_control_mode_it == control_modes_.end() || !new_control_mode_it->second)
  {
    RCLCPP_ERROR(logger, "Can't find control mode \"%s\".", name.c_str());
    return false;
  }

  // Whether we successfully switch controllers in ros2_control
  bool switch_result = false;

  // Deactivate the previous control mode, then switch
  if (current_control_mode_)
  {
    current_control_mode_->get_node()->deactivate();

    const auto previous_control_mode_ = current_control_mode_;
    current_control_mode_ = nullptr;

    // Disable and enable controllers by calling controller manager
    switch_result = switch_controllers(*previous_control_mode_, *new_control_mode_it->second);
  }
  else
  {
    switch_result = switch_controllers(*new_control_mode_it->second);
  }

  if (!switch_result)
  {
    // TODO: Error recovery here
    RCLCPP_ERROR(logger, "Failed to switch ros2_control controllers for control mode switch to \"%s\".", name.c_str());
    return false;
  }

  // Activate the new control mode
  current_control_mode_ = new_control_mode_it->second;
  current_control_mode_->get_node()->activate();
  RCLCPP_INFO(logger, C_MODE "%s activated" C_RESET, snake_to_title(name).c_str());

  return true;
}

void ControlModeManager::update(const rclcpp::Time& now, const rclcpp::Duration& period) const
{
  if (!current_control_mode_)
  {
    RCLCPP_WARN(node_->get_logger(), "ControlModeManager::update(): No mode is active!");
    return;
  }

  current_control_mode_->update(now, period);
}

std::shared_ptr<ControlMode> ControlModeManager::operator[](const std::string& index)
{
  return control_modes_[index];
}

void ControlModeManager::add(const std::string& key, const std::shared_ptr<ControlMode>& value)
{
  // TODO: Implement
  throw std::logic_error("ControlModeManager::add() is not yet implemented. Sorry. Extract it from configure().");
}

void ControlModeManager::reset()
{
  switch_controller_client_ = nullptr;
}

bool ControlModeManager::switch_controllers(const ControlMode& previous, const ControlMode& next) const
{
  if (previous.get_name() == next.get_name())
    return false;

  RCLCPP_DEBUG(node_->get_logger(), "Changing from %s to %s", previous.get_name().c_str(), next.get_name().c_str());

  // TODO: Reimplement 'controllers' param

  // The order of deactivation needs to be opposite to the activation order. This is the reverse to the final order.
  std::vector<std::string> deactivate_controllers_reversed = previous.get_base_params().controllers;
  // Reverse the given order of controllers so they deactivate correctly; in order.
  std::vector<std::string> deactivate_controllers(deactivate_controllers_reversed.size());
  std::reverse_copy(deactivate_controllers_reversed.begin(), deactivate_controllers_reversed.end(),
                    deactivate_controllers.begin());

  // Given order of activated controllers is already correct
  const std::vector<std::string> activate_controllers = next.get_base_params().controllers;

  if (activate_controllers.empty() && deactivate_controllers.empty())
    return true;

  if (!switch_controller_client_->service_is_ready())
  {
    RCLCPP_ERROR(node_->get_logger(), "Controller manager service not available.");
    return false;
  }

  const auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->deactivate_controllers = deactivate_controllers;
  request->activate_controllers = activate_controllers;
  request->strictness = 2;
  request->activate_asap = true;

  auto future = switch_controller_client_->async_send_request(request);

  // TODO: Error recovery when the controller isn't able to switch the controllers.
  return true;
}

bool ControlModeManager::switch_controllers(const ControlMode& next) const
{
  const std::vector<std::string> activate_controllers = next.get_base_params().controllers;

  if (activate_controllers.empty())
    return true;

  if (!switch_controller_client_->service_is_ready())
  {
    RCLCPP_ERROR(node_->get_logger(), "Controller manager service not available.");
    return false;
  }

  const auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers = activate_controllers;
  request->strictness = 2;
  request->activate_asap = true;

  auto future = switch_controller_client_->async_send_request(request);

  // TODO: Error recovery when the controller isn't able to switch the controllers.
  return true;
}

bool ControlModeManager::get_type_for_control_mode(const std::string& name, std::string& control_mode_type) const
{
  // TODO: Check that the parameter hasn't already been defined
  // TODO: Display names
  node_->declare_parameter("control_modes." + name + ".type", rclcpp::ParameterType::PARAMETER_STRING);
  // TODO: Remember that this parameter has already been defined

  rclcpp::Parameter param;
  const auto result = node_->get_parameter("control_modes." + name + ".type", param);

  if (result)
    control_mode_type = param.as_string();
  return result;
}

}  // namespace teleop::internal