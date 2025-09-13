// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "teleop_core/control_modes/ControlModeManager.hpp"

#include <controller_manager_msgs/srv/detail/switch_controller__struct.hpp>
#include <lifecycle_msgs/msg/detail/state__struct.hpp>
#include "teleop_core/colors.hpp"
#include "teleop_core/utilities/utils.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "teleop_core/utilities/get_parameter.hpp"
#include "teleop_core/control_modes/fake_input_collection.hpp"
#include "teleop_core/control_modes/fake_event_collection.hpp"
#include <algorithm>

namespace teleop::internal
{

namespace
{
using control_mode::ControlMode;
using utils::get_parameter;
using utils::get_parameter_or_default;
}  // namespace

void ControlModeManager::configure()
{
  const auto logger = node_->get_logger();


  auto names_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  names_descriptor.name = "control_modes.names";
  names_descriptor.description =
    "The names of all control modes to create, from parameters under control_modes.name.* "
    "for all names provided. Teleop_Modular won't know that parameters in the form "
    "control_modes.name.* exist if name is not in this array.";

  // Declare and get parameter for control modes to spawn by default
  node_->declare_parameter(
    names_descriptor.name, rclcpp::ParameterType::PARAMETER_STRING_ARRAY,
    names_descriptor);
  rclcpp::Parameter control_modes_param;
  node_->get_parameter(names_descriptor.name, control_modes_param);

  if (control_modes_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    RCLCPP_ERROR(logger, "control_modes.names was not set.");
  } else if (control_modes_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    RCLCPP_ERROR(
      logger, "control_modes.names parameter must be a string array, but a %s was given.",
      control_modes_param.get_type_name().c_str());
  }

  const auto control_mode_names = control_modes_param.get_type() ==
    rclcpp::ParameterType::PARAMETER_STRING_ARRAY ?
    control_modes_param.as_string_array() :
    std::vector<std::string>();

  // Pluginlib for loading control modes dynamically
  control_mode_loader_ =
    std::make_unique<pluginlib::ClassLoader<ControlMode>>(
    "teleop_modular",
    "control_mode::ControlMode");

  // List available control mode plugins
  try {
    std::stringstream available_plugins_log{};
    const auto plugins = control_mode_loader_->getDeclaredClasses();
    for (const auto & plugin : plugins) {
      available_plugins_log << "\n\t- " << plugin;
    }

    RCLCPP_DEBUG(logger, "Registered ControlMode plugins:%s", available_plugins_log.str().c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(logger, "Failed to list control mode plugins! what(): %s", ex.what());
    return;
  }

  // Create each control mode according to the given params
  std::stringstream registered_modes_log{};
  for (auto & control_mode_name : control_mode_names) {
    RCLCPP_DEBUG(
      logger, "Attempting to create control mode for name \"%s\"",
      control_mode_name.c_str());

    std::string control_mode_type;
    const std::string pretty_name = snake_to_title(control_mode_name);

    // Get the control mode's plugin type name
    if (!get_type_for_control_mode(control_mode_name, control_mode_type)) {
      RCLCPP_ERROR(
        logger,
        "Failed to find type for control mode \"%s\" in params. Have you defined %s.type in your "
        "parameter file?",
        control_mode_name.c_str(), control_mode_name.c_str());
      registered_modes_log << C_FAIL_QUIET << "\n\t- " << pretty_name << C_FAIL_QUIET <<
        "\t(failed - "
                           << control_mode_name << ".type param missing) " << C_RESET;
      continue;
    }

    // Get the control mode class from pluginlib
    std::shared_ptr<ControlMode> control_mode_class = nullptr;
    try {
      control_mode_class = control_mode_loader_->createSharedInstance(control_mode_type);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        logger, "Failed to find control mode plugin \"%s\" for mode \"%s\"!\nwhat(): %s",
        control_mode_type.c_str(), control_mode_name.c_str(), ex.what());
      registered_modes_log << C_FAIL_QUIET << "\n\t- " << pretty_name << C_FAIL_QUIET
                           << "\t(failed - can't find plugin " << control_mode_type << ") " <<
        C_RESET;
      continue;
    }

    // Create a node for the control mode
    const auto options =
      rclcpp::NodeOptions(node_->get_node_options()).context(
      node_->get_node_base_interface()->get_context());

    // Get common params for control modes
    const auto params = node_->get_node_parameters_interface();
    const auto controllers = get_parameter_or_default<std::vector<std::string>>(
      params, "control_modes." + control_mode_name + ".controllers",
      "The names of ros2_control controllers to use for this control mode.",
      std::vector<std::string>());
    const control_mode::ControlMode::CommonParams common_params{controllers};

    // Set up the control mode
    auto executor = executor_.lock();
    if (!executor) {
      RCLCPP_ERROR(logger, "Failed to get the executor when setting up control modes!");
      throw std::runtime_error("Failed to get the executor when setting up control modes!");
    }

    control_mode_class->init(
      control_mode_name, node_->get_namespace(), options, executor,
      common_params);
    executor.reset();  // Release executor

    registered_modes_log << "\n\t- " << pretty_name << C_QUIET << "\t: " << control_mode_type <<
      C_RESET;
    control_modes_[control_mode_name] = control_mode_class;
  }

  RCLCPP_INFO(logger, C_TITLE "Control Modes:" C_RESET "%s\n", registered_modes_log.str().c_str());

  // Configure each control mode
  for (const auto & [name, control_mode] : control_modes_) {
    if (!control_mode) {
      continue;
    }

    control_mode->get_node()->configure();
    // TODO: Configure inputs if they are available
  }
}

void ControlModeManager::activate_initial_control_mode()
{  // Activate the first control mode in the list
  if (!control_modes_.empty()) {
    set_control_mode(control_modes_.begin()->first);
  }
}

bool ControlModeManager::switch_control_mode(
  const std::vector<std::string> & activate,
  const std::vector<std::string> & deactivate)
{
  const auto logger = node_->get_logger();

  // First populate activate_names and deactivate_names to filter any duplicates
  const std::set<std::string> activate_names(activate.begin(), activate.end());
  const std::set<std::string> deactivate_names(deactivate.begin(), deactivate.end());

  // for (const auto& name : activate_names) {
  //   deactivate_names.remove(name);
  // }

  std::vector<std::string> activate_controllers{};
  std::vector<std::string> activate_controllers_set{};

  // Get the names of controllers to activate first


  std::vector<std::string> deactivate_controllers_reversed{};
  std::set<std::string> deactivate_controllers_set{};
  std::set<std::string> common_controllers_set;

  // Deactivate control modes
  for (const auto& name : deactivate) {
    // Check if this name is being activated, and ignore if it is
    if (activate_names.find(name) != activate_names.end()) {
      RCLCPP_WARN(logger, "Tried to deactivate control mode \"%s\", but it was also in the list to be activated. Skipping...", name.c_str());
      continue;
    }

    // Get the control mode to deactivate
    const auto it = control_modes_.find(name);
    if (it == control_modes_.end()) {
      RCLCPP_ERROR(logger, "Unable to find control mode with name \"%s\" to deactivate.", name.c_str());
      continue;
    }
    const auto& mode = it->second;

    // TODO: Check that this control mode isn't also being activated

    if (!mode->is_active()) {
      continue;
    }

    mode->get_node()->deactivate();

    // Add controllers to deactivate
    for (const auto & controller : mode->get_controllers()) {
      if (deactivate_controllers_set.find(controller) != deactivate_controllers_set.end()) {
        continue;
      }

      // TODO: Make sure we dont deactivate controllers also being activated

      const auto insert_result = deactivate_controllers_set.insert(controller);

      // Only add to the list of deactivations if the insertion actually took place
      // (i.e. this is the first time the control mode has been seen)
      if (insert_result.second)
        deactivate_controllers_reversed.push_back(controller);
    }
  }

  for (const auto& name : activate_names) {
    // Get the control mode
    const auto it = control_modes_.find(name);
    if (it == control_modes_.end()) {
      RCLCPP_ERROR(logger, "Unable to find control mode with name \"%s\".", name.c_str());
      continue;
    }
    const auto& mode = *it;

    // Skip anything already active
    if (mode->is_active()) {
      RCLCPP_WARN(logger, "Tried to activate control mode with name \"%s\", but it is already activated. Skipping...", name.c_str());
      continue;
    }

    mode.



  }


}



bool ControlModeManager::set_control_mode(const std::string & name)
{
  const auto logger = node_->get_logger();

  // Find the control mode from the name
  const auto new_control_mode_it = std::find_if(
    control_modes_.begin(), control_modes_.end(),
    [name](const std::pair<std::string, std::shared_ptr<ControlMode>> & pair) {
      return pair.first == name;
    });

  // Ensure the given control mode exists
  if (new_control_mode_it == control_modes_.end() || !new_control_mode_it->second) {
    RCLCPP_ERROR(logger, "Can't find control mode \"%s\".", name.c_str());
    return false;
  }
  const auto new_control_mode = new_control_mode_it->second;

  std::vector<std::string> new_controllers = new_control_mode->get_controllers();
  const std::set<std::string> new_controllers_set(new_controllers.begin(), new_controllers.end());

  // Whether we successfully switch controllers in ros2_control
  bool switch_result = false;

  // Deactivate the previous control mode, then switch
  std::vector<std::string> deactivate_controllers_reversed;
  std::set<std::string> deactivate_controllers_set;
  std::set<std::string> common_controllers_set;

  // Get controllers to deactivate
  // TODO: Preserve order of deactivations (deactivating 'ac' and 'abc' -> 'abc', not 'acb')
  for (const auto & [deactivate_name, control_mode] : control_modes_) {
    if (control_mode == new_control_mode) {
      continue;
    }

    if (!control_mode->is_active()) {
      continue;
    }

    control_mode->get_node()->deactivate();

    for (const auto & controller : control_mode->get_controllers()) {
      if (deactivate_controllers_set.find(controller) != deactivate_controllers_set.end()) {
        continue;
      }

      deactivate_controllers_set.insert(controller);

      // Filter out anything common with the new control modes controllers
      if (new_controllers_set.find(controller) != new_controllers_set.end()) {
        common_controllers_set.insert(controller);
        continue;
      }

      deactivate_controllers_reversed.push_back(controller);
    }
  }

  // Construct controllers_to_activate by filtering out controllers common to those being deactivated
  std::vector<std::string> controllers_to_activate;
  controllers_to_activate.reserve(new_controllers.size() - common_controllers_set.size());
  for (const auto & controller : new_controllers) {
    if (common_controllers_set.find(controller) != common_controllers_set.end()) {
      continue;
    }

    controllers_to_activate.push_back(controller);
  }

  // The order of deactivation needs to be opposite to the activation order. This is the reverse to the final order.
  // Reverse the given order of controllers so they deactivate correctly; in order.
  std::vector<std::string> controllers_to_deactivate(deactivate_controllers_reversed.size());
  std::reverse_copy(
    deactivate_controllers_reversed.begin(), deactivate_controllers_reversed.end(),
    controllers_to_deactivate.begin());

  switch_result = switch_controllers(controllers_to_deactivate, controllers_to_activate);

  // Disable and enable controllers by calling controller manager
  if (!switch_result) {
    std::stringstream log_msg{};
    log_msg << "\nControllers to deactivate:";
    for (const auto & controller : controllers_to_deactivate) {
      log_msg << "\n  - " << controller;
    }
    log_msg << "\nControllers to activate:";
    for (const auto & controller : controllers_to_activate) {
      log_msg << "\n  - " << controller;
    }

    // TODO: Error recovery here
    RCLCPP_ERROR(
      logger, "Failed to switch ros2_control controllers for control mode switch to \"%s\":%s",
      name.c_str(),
      log_msg.str().c_str());
    return false;
  }

  // Activate the new control mode
  //  current_control_mode_ = new_control_mode_it->second;
  new_control_mode->get_node()->activate();

  RCLCPP_INFO(logger, C_MODE "%s activated" C_RESET, snake_to_title(name).c_str());

  return true;
}

void ControlModeManager::update(const rclcpp::Time & now, const rclcpp::Duration & period) const
{
  bool any_control_mode_updated = false;

  for (const auto & [name, control_mode] : control_modes_) {
    if (!control_mode->is_active()) {
      continue;
    }

    control_mode->on_update(now, period);
    any_control_mode_updated = true;
  }

  if (!any_control_mode_updated) {
    const auto logger = node_->get_logger();
    std::stringstream log;

    for (const auto & [name, control_mode] : control_modes_) {
      log << "\n  - " << name << " is " << control_mode->get_lifecycle_state().label();
    }

    RCLCPP_WARN_THROTTLE(logger, *node_->get_clock(), 2000, "ControlModeManager::update(): No mode is active!%s", );
  }
}

std::shared_ptr<ControlMode> ControlModeManager::operator[](const std::string & index)
{
  return control_modes_[index];
}

void ControlModeManager::add(const std::string & key, const std::shared_ptr<ControlMode> & value)
{
  // TODO: Implement
  throw std::logic_error(
          "ControlModeManager::add() is not yet implemented. Sorry. Extract it from configure().");
}

void ControlModeManager::reset()
{
  switch_controller_client_ = nullptr;
}


bool ControlModeManager::get_type_for_control_mode(
  const std::string & name,
  std::string & control_mode_type) const
{
  // TODO: Check that the parameter hasn't already been defined
  // TODO: Display names
  node_->declare_parameter(
    "control_modes." + name + ".type",
    rclcpp::ParameterType::PARAMETER_STRING);
  // TODO: Remember that this parameter has already been defined

  rclcpp::Parameter param;
  const auto result = node_->get_parameter("control_modes." + name + ".type", param);

  if (result) {
    control_mode_type = param.as_string();
  }
  return result;
}

void ControlModeManager::link_inputs(const InputManager::Props& previous, InputManager::Props& next, const InputPipelineBuilder::DeclaredNames& declared_names) {
  // No inputs to provide!
}

void ControlModeManager::declare_input_names(InputPipelineBuilder::DeclaredNames& names)
{
  auto fake_buttons = FakeInputCollection<control_mode::Button>();
  auto fake_axes = FakeInputCollection<control_mode::Axis>();

  auto fake_events = FakeEventCollection(fake_buttons);

  control_mode::Inputs control_mode_inputs {
    fake_buttons,
    fake_axes,
    fake_events
  };

  // Provide fake inputs to the control mode in order to extract the input names they want to use
  for (auto& [name, mode] : control_modes_) {
    // TODO: Make sure we don't try configure any errored out modes
    mode->configure_inputs(control_mode_inputs);
  }

  names.button_names.insert(fake_buttons.get_names().begin(), fake_buttons.get_names().end());
  names.axis_names.insert(fake_axes.get_names().begin(), fake_axes.get_names().end());
}

void ControlModeManager::on_inputs_available(InputManager::Hardened& inputs)
{
  auto control_mode_inputs = control_mode::Inputs {
    inputs.buttons,
    inputs.axes,
    events_
  };

  // Provide the real inputs after supplying fake inputs in declare_input_names()
  for (auto& [name, mode] : control_modes_) {
    mode->configure_inputs(control_mode_inputs);
  }
}

}  // namespace teleop::internal
