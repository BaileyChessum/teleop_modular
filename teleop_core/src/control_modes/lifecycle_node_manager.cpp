// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#include "teleop_core/control_modes/lifecycle_node_manager.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include "teleop_core/utilities/get_parameter.hpp"

namespace teleop
{

namespace
{
using utils::get_parameter_or_default;

constexpr const char * kLifecycleNodeManagerName = "lifecycle_node_manager";
}  // namespace

LifecycleNodeManager::LifecycleNodeManager(rclcpp::Node::SharedPtr node)
: OrderedResourceManager(std::move(node), kLifecycleNodeManagerName)
{
  auto parameters_interface = context().node->get_node_parameters_interface();

  params_.log_lifecycle_nodes = get_parameter_or_default<bool>(
    parameters_interface,
    "log_lifecycle_nodes",
    "Whether to log debug info about lifecycle node activations.",
    false);
  params_.impatient = get_parameter_or_default<bool>(
    parameters_interface,
    "lifecycle_node_manager.impatient",
    "Whether to wait for lifecycle node requests to finish before polling for a newer desired state.",
    false);
  params_.reasonable_timeout = get_parameter_or_default<double>(
    parameters_interface,
    "lifecycle_node_manager.reasonable_timeout",
    "Time in seconds to keep waiting for a lifecycle node service response before switching to polling mode.",
    3.0);
  params_.connection_retry_count = get_parameter_or_default<int64_t>(
    parameters_interface,
    "lifecycle_node_manager.connection_retry_count",
    "Number of times to check if lifecycle node services are available before giving up. Set to -1 to never stop trying.",
    50);
  params_.connection_retry_rate = get_parameter_or_default<double>(
    parameters_interface,
    "lifecycle_node_manager.connection_retry_rate",
    "The rate at which lifecycle node service availability should be retried, in hz.",
    10.0);

  start_worker();
}

bool LifecycleNodeManager::register_lifecycle_nodes_for_ids(
  const std::vector<std::reference_wrapper<const std::vector<std::string>>> & lifecycle_node_names_for_ids)
{
  if (!register_resources_for_ids(lifecycle_node_names_for_ids)) {
    return false;
  }

  clients_by_node_name_.clear();
  for (size_t i = 0; i < context().ordering.size(); ++i) {
    const auto & node_name = context().ordering[i];
    clients_by_node_name_[node_name] = ServiceClients{
      context().node->create_client<lifecycle_msgs::srv::ChangeState>(node_name + "/change_state"),
      context().node->create_client<lifecycle_msgs::srv::GetState>(node_name + "/get_state"),
    };
  }

  return true;
}

bool LifecycleNodeManager::should_log_resources() const
{
  return params_.log_lifecycle_nodes;
}

const char * LifecycleNodeManager::resource_kind_plural() const
{
  return "lifecycle nodes";
}

bool LifecycleNodeManager::apply_resource_change(
  const std::vector<std::string> & lifecycle_nodes_to_deactivate,
  const std::vector<std::string> & lifecycle_nodes_to_activate)
{
  for (const auto & node_name : lifecycle_nodes_to_deactivate) {
    if (!ensure_node_inactive(node_name)) {
      return false;
    }
  }

  for (const auto & node_name : lifecycle_nodes_to_activate) {
    if (!ensure_node_active(node_name)) {
      return false;
    }
  }

  return true;
}

OrderedResourceManager::RefreshResult LifecycleNodeManager::refresh_current_active_resources()
{
  std::set<size_t> active_ids{};

  for (size_t id = 0; id < context().ordering.size(); ++id) {
    const auto & node_name = context().ordering[id];
    const auto state = get_state_for_node(node_name);
    if (!state.has_value()) {
      return RefreshResult::Failed;
    }

    if (should_log_resources()) {
      RCLCPP_INFO(
        context().logger,
        "  - \"%s\" (id: %lu)\t: %s",
        node_name.c_str(),
        id,
        state->label.c_str());
    }

    if (state->id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      active_ids.insert(id);
    }
  }

  return replace_current_active_resources(std::move(active_ids));
}

std::optional<lifecycle_msgs::msg::State> LifecycleNodeManager::get_state_for_node(const std::string & node_name)
{
  const auto clients_it = clients_by_node_name_.find(node_name);
  if (clients_it == clients_by_node_name_.end()) {
    RCLCPP_ERROR(context().logger, "No lifecycle service clients were configured for node \"%s\".", node_name.c_str());
    return std::nullopt;
  }

  const auto & client = clients_it->second.get_state;
  const auto service_name = node_name + "/get_state";
  const int max_attempts = params_.connection_retry_count;

  for (int i = 0; (max_attempts < 0 || i < max_attempts) && !client->service_is_ready() && !stop_requested(); ++i) {
    RCLCPP_WARN_THROTTLE(
      context().logger,
      *context().node->get_clock(),
      2000,
      "%s service not currently available.",
      service_name.c_str());
    wait_for_stop_or(std::chrono::duration<double>(1.0 / params_.connection_retry_rate));
  }

  if (stop_requested()) {
    return std::nullopt;
  }

  if (!client->service_is_ready()) {
    RCLCPP_ERROR(
      context().logger,
      "%s not available after %d attempts. Stopping attempts.",
      service_name.c_str(),
      max_attempts);
    return std::nullopt;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  std::optional future(client->async_send_request(request));
  auto response = wait_for_response(
    future,
    "lifecycle get_state",
    params_.impatient,
    params_.reasonable_timeout);

  if (!response) {
    return std::nullopt;
  }

  return response->current_state;
}

bool LifecycleNodeManager::ensure_node_active(const std::string & node_name)
{
  auto state = get_state_for_node(node_name);
  if (!state.has_value()) {
    return false;
  }

  switch (state->id) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      return true;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      if (!request_transition(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "activate")) {
        return false;
      }
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      if (!request_transition(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "configure")) {
        return false;
      }
      state = get_state_for_node(node_name);
      if (!state.has_value()) {
        return false;
      }
      if (state->id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        return true;
      }
      if (state->id != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_ERROR(
          context().logger,
          "Lifecycle node \"%s\" ended configure in unexpected state \"%s\".",
          node_name.c_str(),
          state->label.c_str());
        return false;
      }
      if (!request_transition(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "activate")) {
        return false;
      }
      break;
    default:
      RCLCPP_ERROR(
        context().logger,
        "Lifecycle node \"%s\" is in unsupported state \"%s\" for activation.",
        node_name.c_str(),
        state->label.c_str());
      return false;
  }

  state = get_state_for_node(node_name);
  return state.has_value() && state->id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

bool LifecycleNodeManager::ensure_node_inactive(const std::string & node_name)
{
  const auto state = get_state_for_node(node_name);
  if (!state.has_value()) {
    return false;
  }

  switch (state->id) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      if (!request_transition(node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, "deactivate")) {
        return false;
      }
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      return true;
    default:
      RCLCPP_ERROR(
        context().logger,
        "Lifecycle node \"%s\" is in unsupported state \"%s\" for deactivation.",
        node_name.c_str(),
        state->label.c_str());
      return false;
  }

  const auto final_state = get_state_for_node(node_name);
  return final_state.has_value() && final_state->id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

bool LifecycleNodeManager::request_transition(
  const std::string & node_name,
  uint8_t transition_id,
  const char * transition_name)
{
  const auto clients_it = clients_by_node_name_.find(node_name);
  if (clients_it == clients_by_node_name_.end()) {
    RCLCPP_ERROR(context().logger, "No lifecycle service clients were configured for node \"%s\".", node_name.c_str());
    return false;
  }

  const auto & client = clients_it->second.change_state;
  const auto service_name = node_name + "/change_state";
  const int max_attempts = params_.connection_retry_count;

  for (int i = 0; (max_attempts < 0 || i < max_attempts) && !client->service_is_ready() && !stop_requested(); ++i) {
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

  if (!client->service_is_ready()) {
    RCLCPP_ERROR(
      context().logger,
      "%s not available after %d attempts. Stopping attempts.",
      service_name.c_str(),
      max_attempts);
    return false;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition_id;

  std::optional future(client->async_send_request(request));
  auto response = wait_for_response(
    future,
    "lifecycle change_state",
    params_.impatient,
    params_.reasonable_timeout);

  if (!response) {
    return false;
  }

  if (!response->success) {
    RCLCPP_ERROR(
      context().logger,
      "Failed to %s lifecycle node \"%s\" via %s.",
      transition_name,
      node_name.c_str(),
      service_name.c_str());
    return false;
  }

  return true;
}

}  // namespace teleop
