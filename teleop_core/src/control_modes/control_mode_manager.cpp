// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "teleop_core/control_modes/control_mode_manager.hpp"

#include <controller_manager_msgs/srv/detail/switch_controller__struct.hpp>
#include <lifecycle_msgs/msg/detail/state__struct.hpp>
#include "teleop_core/colors.hpp"
#include "teleop_core/utilities/utils.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "teleop_core/utilities/get_parameter.hpp"
#include "teleop_core/control_modes/fake_input_collection.hpp"
#include "teleop_core/control_modes/fake_event_collection.hpp"
#include <algorithm>
#include <stdexcept>
#include <iomanip>  //< for std::setw, std::setfill

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

    const auto params = node_->get_node_parameters_interface();
    const auto display_name = get_parameter_or_default<std::string>(
        params, "control_modes." + control_mode_name + ".display_name",
        "The name to use in user-facing logs for this control mode.",
        snake_to_title(control_mode_name));

    // Get the control mode's plugin type name
    if (!get_type_for_control_mode(control_mode_name, control_mode_type)) {
      RCLCPP_ERROR(
        logger,
        "Failed to find type for control mode \"%s\" in params. Have you defined %s.type in your "
        "parameter file?",
        control_mode_name.c_str(), control_mode_name.c_str());
      registered_modes_log << C_FAIL_QUIET << "\n\t- " << display_name << C_FAIL_QUIET <<
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
      registered_modes_log << C_FAIL_QUIET << "\n\t- " << display_name << C_FAIL_QUIET
                           << "\t(failed - can't find plugin " << control_mode_type << ") " <<
        C_RESET;
      continue;
    }

    // Create a node for the control mode
    const auto options =
      rclcpp::NodeOptions(node_->get_node_options()).context(
      node_->get_node_base_interface()->get_context());

    // Get common params for control modes
    const auto controllers = get_parameter_or_default<std::vector<std::string>>(
      params, "control_modes." + control_mode_name + ".controllers",
      "The names of ros2_control controllers to use for this control mode.",
      std::vector<std::string>());
    const auto start_active = get_parameter_or_default<bool>(
        params, "control_modes." + control_mode_name + ".active",
        "Whether this control mode should start active.",
        false);

    const control_mode::ControlMode::CommonParams common_params{
      controllers,
      start_active,
      display_name
    };

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

    registered_modes_log << "\n\t- " << display_name << C_QUIET << "\t: " << control_mode_type <<
      C_RESET;
    control_modes_[control_mode_name] = control_mode_class;
  }

  RCLCPP_INFO(logger, C_TITLE "Control Modes:" C_RESET "%s\n", registered_modes_log.str().c_str());

  // Configure each control mode
  for (const auto & [name, control_mode] : control_modes_) {
    if (!control_mode)
      throw std::invalid_argument("control_mode \"" + name + "\" was nullptr.");

    control_mode->get_node()->configure();
    // TODO: Configure inputs if they are available
  }

  // Put all the control modes into a vector, so that they can be indexed by id
  control_modes_by_id_.clear();
  control_modes_by_id_.reserve(control_modes_.size());
  size_t i = 0;
  for (const auto & [name, control_mode] : control_modes_) {
    control_modes_by_id_.emplace_back(control_mode);
    name_to_cm_id_[name] = i;
    ++i;
  }

  // Set up the controller manager manager with the given controller names
  // Get the names of every control mode in a vector
  std::vector<std::reference_wrapper<const std::vector<std::string>>> controllers_for_cm_ids{};
  controllers_for_cm_ids.reserve(control_modes_.size());

  for (const auto control_mode : control_modes_by_id_) {
    if (!control_mode)
      throw std::invalid_argument("control_mode \"" + control_mode->get_name() + "\" was nullptr.");

    controllers_for_cm_ids.emplace_back(std::ref(control_mode->get_controllers()));
  }

  controllers_.register_controllers_for_ids(controllers_for_cm_ids);
  // TODO: Gracefully reconfigure the controller manager manager if a new control mode gets added dynamically
}

void ControlModeManager::activate_initial_control_modes()
{  // Activate the first control mode in the list
  std::vector<std::string> activate;

  for (auto mode : control_modes_by_id_) {
    if (mode->get_common_params().start_active) {
      activate.push_back(mode->get_name());
    }
  }

  if (!activate.empty()) {
    switch_control_mode({}, activate);
  }
  else {
    const auto logger = node_->get_logger();
    RCLCPP_WARN(logger,
                "No control modes start activated. \nDid you mean to set the parameter:\n"
                "\tcontrol_modes.<name>.active: true\nfor one of your control modes?");
  }

  state_publisher_.publish_control_mode_states(node_->now());
}

bool ControlModeManager::switch_control_mode(
  const std::vector<std::string> & deactivate,
  const std::vector<std::string> & activate)
{
  const auto logger = node_->get_logger();

  // Turn names into cm ids
  std::vector<size_t> deactivate_ids{};
  std::vector<size_t> activate_ids{};
  std::set<size_t> activate_ids_set{};  //< Lets us filter out deactivate_ids elements that are also in activated

  std::vector<std::shared_ptr<ControlMode>> deactivate_modes{};
  std::vector<std::shared_ptr<ControlMode>> activate_modes{};

  for (const auto& name : activate) {
    const auto it = name_to_cm_id_.find(name);
    if (it == name_to_cm_id_.end()) {
      RCLCPP_ERROR(logger, "Unable to find control mode with name \"%s\" to activate.", name.c_str());
      continue;
    }
    const auto& cm_id = it->second;

    // Prevent this id from being deactivated
    activate_ids_set.insert(cm_id);

    // Only activate if not already active
    if (control_modes_by_id_[cm_id]->is_active())
      continue;

    activate_ids.emplace_back(cm_id);
  }

  for (const auto& name : deactivate) {
    const auto it = name_to_cm_id_.find(name);
    if (it == name_to_cm_id_.end()) {
      RCLCPP_ERROR(logger, "Unable to find control mode with name \"%s\" to deactivate.", name.c_str());
      continue;
    }
    const auto& cm_id = it->second;

    // Only deactivate if currently active
    if (!control_modes_by_id_[cm_id]->is_active())
      continue;

    // Filter out any control modes also being activated
    if (activate_ids_set.find(cm_id) != activate_ids_set.end())
      continue;

    deactivate_ids.emplace_back(cm_id);
  }

  // Eagerly change active controllers in ros2_control in a separate worker thread, expecting all control mode state
  // transitions will succeed
  controllers_.switch_active(deactivate_ids, activate_ids);

  // These store the ids of modes to log after all switches have occurred:
  std::vector<size_t> deactivate_log_ids{};
  std::vector<size_t> activate_log_ids{};
  deactivate_log_ids.reserve(deactivate_ids.size());
  activate_log_ids.reserve(activate_ids.size());

  // Deactivate control modes
  std::vector<size_t> failed_deactivation_ids{};
  for (const auto& cm_id : deactivate_ids)
  {
    // TODO: Check the success of the transition, and active respective ros2_control controllers on failure
    auto result = control_modes_by_id_[cm_id]->get_node()->deactivate();

    // Check for failures
    if (result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      failed_deactivation_ids.emplace_back(cm_id);
      RCLCPP_ERROR(logger, C_MODE "Failed to deactivate control mode \"%s\"." C_RESET,
                   control_modes_by_id_[cm_id]->get_name().c_str());
    }
    else {
      RCLCPP_DEBUG(logger, C_DEACTIVATED "%s deactivated" C_RESET,
                   control_modes_by_id_[cm_id]->get_name().c_str());
      deactivate_log_ids.emplace_back(cm_id);
    }
  }

  // Activate control modes
  std::vector<size_t> failed_activation_ids{};
  for (const auto& cm_id : activate_ids)
  {
    auto result = control_modes_by_id_[cm_id]->get_node()->activate();

    // Check for failures
    if (result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      failed_activation_ids.emplace_back(cm_id);
      RCLCPP_ERROR(logger, C_MODE "Failed to activate control mode \"%s\"." C_RESET,
                   control_modes_by_id_[cm_id]->get_name().c_str());
    }
    else {
      RCLCPP_DEBUG(logger, C_MODE "%s activated" C_RESET,
                   control_modes_by_id_[cm_id]->get_name().c_str());
      activate_log_ids.emplace_back(cm_id);
    }
  }

  // Fancy log statement

  // Get longest name in log
  size_t longest_name_length = 0;
  for (const auto& cm_id : activate_ids) {
    auto size = control_modes_by_id_[cm_id]->get_common_params().display_name.size();
    if (size > longest_name_length)
      longest_name_length = size;
  }
  for (const auto& cm_id : deactivate_ids) {
    auto size = control_modes_by_id_[cm_id]->get_common_params().display_name.size();
    if (size > longest_name_length)
      longest_name_length = size;
  }

  // Log every name
  int name_col_width = static_cast<int>(longest_name_length) + 1;
  std::stringstream log;
  for (const auto& cm_id : deactivate_ids) {
    log << "\n\t" C_DEACTIVATED_QUIET_BOLD "- " C_DEACTIVATED;
    // Log name left-aligned in column of width = name_col_width
    log << std::left << std::setw(name_col_width) << std::setfill(' ');
    log << control_modes_by_id_[cm_id]->get_common_params().display_name;
    log << C_DEACTIVATED_QUIET "deactivated" C_RESET;
  }
  for (const auto& cm_id : activate_ids) {
    log << "\n\t" C_MODE_QUIET_BOLD "+ " C_MODE;
    // Log name left-aligned in column of width = name_col_width
    log << std::left << std::setw(name_col_width) << std::setfill(' ');
    log << control_modes_by_id_[cm_id]->get_common_params().display_name;
    log << C_MODE_QUIET "activated" C_RESET;
  }

  RCLCPP_INFO(logger, C_QUIET_QUIET "Switched control modes:" C_RESET "%s", log.str().c_str());

  // Publish state
  state_publisher_.publish_control_mode_states(node_->now());

  // Send another controller change request on any transition failure
  if (failed_activation_ids.size() > 0 || failed_deactivation_ids.size() > 0) {
    RCLCPP_WARN(logger,
                "Some control mode activations and deactivations failed. A second switch_controller request is being "
                "made to compensate.");
    controllers_.switch_active(failed_deactivation_ids, failed_activation_ids);
    return false;
  }

  return true;
}

bool ControlModeManager::set_control_mode(const std::string & name)
{
  const auto logger = node_->get_logger();

  // Get every node name excluding the given name
  std::vector<std::string> deactivate_names{};
  deactivate_names.reserve(control_modes_by_id_.size() - 1);
  for (auto mode : control_modes_by_id_) {
    if (mode->get_name() == name)
      continue;

    deactivate_names.emplace_back(mode->get_name());
  }

  const std::vector<std::string> activate_names{ name };

  return switch_control_mode(deactivate_names, activate_names);
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

    RCLCPP_WARN_THROTTLE(logger, *node_->get_clock(), 2000, "ControlModeManager::update(): No mode is active!%s", log.str().c_str());
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
  next = previous;
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

  state_publisher_.configure_inputs(control_mode_inputs);

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

  state_publisher_.configure_inputs(control_mode_inputs);
  state_publisher_.publish_state(node_->now());

  // Provide the real inputs after supplying fake inputs in declare_input_names()
  for (auto& [name, mode] : control_modes_) {
    mode->configure_inputs(control_mode_inputs);
  }
}

}  // namespace teleop::internal
