// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#include "teleop_core/control_modes/controller_manager_manager.hpp"

#include <cmath>
#include <set>

#include "teleop_core/utilities/get_parameter.hpp"

namespace
{

inline builtin_interfaces::msg::Duration to_duration(double seconds)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(std::floor(seconds));
  duration.nanosec = static_cast<uint32_t>(std::round((seconds - duration.sec) * 1e9));

  if (duration.nanosec == 1000000000) {
    duration.sec += 1;
    duration.nanosec = 0;
  }
  return duration;
}

}  // namespace

namespace teleop
{

namespace
{
using utils::get_parameter_or_default;
constexpr const char * kControllerManagerManagerName = "controller_manager_manager";
}  // namespace

ControllerManagerManager::ControllerManagerManager(rclcpp::Node::SharedPtr node)
: OrderedResourceManager(std::move(node), kControllerManagerManagerName)
{
  auto parameters_interface = context().node->get_node_parameters_interface();

  params_.controller_manager = get_parameter_or_default<std::string>(
    parameters_interface,
    "controller_manager",
    "The name of the controller manager to use when switching controllers for control modes. Don't include a '/' at the end.",
    "/controller_manager");
  params_.log_controllers = get_parameter_or_default<bool>(
    parameters_interface,
    "log_controllers",
    "Whether to log debug info about controller activations.",
    false);
  params_.impatient = get_parameter_or_default<bool>(
    parameters_interface,
    "controller_manager_manager.impatient",
    "Whether to wait for controller manager requests to finish before polling for a newer desired state.",
    false);
  params_.strictness = get_parameter_or_default<int64_t>(
    parameters_interface,
    "controller_manager_manager.strictness",
    "strictness value to use in /switch_controller service messages.\n"
    "The strictness (STRICT=2, BEST_EFFORT=1, AUTO=3, or FORCE_AUTO=4). STRICT by default.",
    2);
  params_.activate_asap = get_parameter_or_default<bool>(
    parameters_interface,
    "controller_manager_manager.activate_asap",
    "activate_asap value to use in /switch_controller service messages.",
    true);
  params_.timeout = to_duration(get_parameter_or_default<double>(
      parameters_interface,
      "controller_manager_manager.timeout",
      "timeout value (in seconds) to use in /switch_controller service messages. Zero for infinite.",
      0.0));
  params_.reasonable_timeout = get_parameter_or_default<double>(
    parameters_interface,
    "controller_manager_manager.reasonable_timeout",
    "Time in seconds to keep waiting for a controller manager service response before switching to polling mode.",
    3.0);
  params_.connection_retry_count = get_parameter_or_default<int64_t>(
    parameters_interface,
    "controller_manager_manager.connection_retry_count",
    "Number of times to check if controller manager services are available before giving up. Set to -1 to never stop trying.",
    50);
  params_.connection_retry_rate = get_parameter_or_default<double>(
    parameters_interface,
    "controller_manager_manager.connection_retry_rate",
    "The rate at which controller manager service availability should be retried, in hz.",
    10.0);

  switch_controller_client_ =
    context().node->create_client<controller_manager_msgs::srv::SwitchController>(
    params_.controller_manager + "/switch_controller");
  list_controllers_client_ =
    context().node->create_client<controller_manager_msgs::srv::ListControllers>(
    params_.controller_manager + "/list_controllers");

  start_worker();
}

bool ControllerManagerManager::register_controllers_for_ids(
  const std::vector<std::reference_wrapper<const std::vector<std::string>>> & controller_names_for_ids)
{
  return register_resources_for_ids(controller_names_for_ids);
}

bool ControllerManagerManager::should_log_resources() const
{
  return params_.log_controllers;
}

const char * ControllerManagerManager::resource_kind_plural() const
{
  return "controllers";
}

bool ControllerManagerManager::apply_resource_change(
  const std::vector<std::string> & controllers_to_deactivate,
  const std::vector<std::string> & controllers_to_activate)
{
  if (controllers_to_deactivate.empty() && controllers_to_activate.empty()) {
    return true;
  }

  const auto service_name = params_.controller_manager + "/switch_controller";
  const int max_attempts = params_.connection_retry_count;
  for (int i = 0; (max_attempts < 0 || i < max_attempts) && !switch_controller_client_->service_is_ready() && !stop_requested(); ++i) {
    RCLCPP_WARN_THROTTLE(
      context().logger,
      *context().node->get_clock(),
      2000,
      "%s service not currently available.",
      service_name.c_str());
    wait_for_stop_or(std::chrono::duration<double>(1.0 / params_.connection_retry_rate));
  }

  if (stop_requested()) {
    return false;
  }

  if (!switch_controller_client_->service_is_ready()) {
    RCLCPP_ERROR(
      context().logger,
      "%s not available after %d attempts. Stopping attempts.",
      service_name.c_str(),
      max_attempts);
    return false;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->deactivate_controllers = controllers_to_deactivate;
  request->activate_controllers = controllers_to_activate;
  request->strictness = params_.strictness;
  request->activate_asap = params_.activate_asap;
  request->timeout = params_.timeout;

  std::optional future(switch_controller_client_->async_send_request(request));
  auto response = wait_for_response(
    future,
    "switch_controller",
    params_.impatient,
    params_.reasonable_timeout);

  if (!response) {
    return false;
  }

  if (!response->ok) {
    RCLCPP_ERROR(context().logger, "Failed to switch controllers.");
    return false;
  }

  if (should_log_resources()) {
    RCLCPP_INFO(context().logger, "Successfully switched controllers.\n");
  }

  return true;
}

OrderedResourceManager::RefreshResult ControllerManagerManager::refresh_current_active_resources()
{
  const auto service_name = params_.controller_manager + "/list_controllers";
  const int max_attempts = params_.connection_retry_count;
  for (int i = 0; (max_attempts < 0 || i < max_attempts) && !list_controllers_client_->service_is_ready() && !stop_requested(); ++i) {
    RCLCPP_WARN_THROTTLE(
      context().logger,
      *context().node->get_clock(),
      2000,
      "%s service not currently available.",
      service_name.c_str());
    wait_for_stop_or(std::chrono::duration<double>(1.0 / params_.connection_retry_rate));
  }

  if (stop_requested()) {
    return RefreshResult::Failed;
  }

  if (!list_controllers_client_->service_is_ready()) {
    RCLCPP_ERROR(
      context().logger,
      "%s not available after %d attempts. Stopping attempts.",
      service_name.c_str(),
      max_attempts);
    return RefreshResult::Failed;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  std::optional future(list_controllers_client_->async_send_request(request));
  auto response = wait_for_response(
    future,
    "list_controllers",
    params_.impatient,
    params_.reasonable_timeout);

  if (!response) {
    return RefreshResult::Failed;
  }

  std::set<size_t> actual_active_controllers{};
  if (should_log_resources()) {
    RCLCPP_INFO(context().logger, "Actual controller states from ros2_control:");
  }

  for (const auto & controller : response->controller) {
    if (!context().ordering.contains(controller.name)) {
      if (should_log_resources()) {
        RCLCPP_INFO(
          context().logger,
          "  - \"%s\" (not defined in teleop)\t: %s",
          controller.name.c_str(),
          controller.state.c_str());
      }
      continue;
    }

    const auto is_active = controller.state == "active";
    const auto is_inactive = controller.state == "inactive";
    const auto id = context().ordering[controller.name];

    if (should_log_resources()) {
      RCLCPP_INFO(
        context().logger,
        "  - \"%s\" (id: %lu)\t: %s",
        controller.name.c_str(),
        id,
        controller.state.c_str());
    }

    if (!is_active && !is_inactive) {
      RCLCPP_ERROR(
        context().logger,
        "Controller \"%s\" is in state \"%s\". Treating it as inactive...",
        controller.name.c_str(),
        controller.state.c_str());
      continue;
    }

    if (is_active) {
      actual_active_controllers.insert(id);
    }
  }

  return replace_current_active_resources(std::move(actual_active_controllers));
}

}  // namespace teleop
