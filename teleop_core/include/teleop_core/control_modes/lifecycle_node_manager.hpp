// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#ifndef TELEOP_CORE__CONTROL_MODES__LIFECYCLE_NODE_MANAGER_HPP_
#define TELEOP_CORE__CONTROL_MODES__LIFECYCLE_NODE_MANAGER_HPP_

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/node.hpp>

#include "teleop_core/control_modes/ordered_resource_manager.hpp"

namespace teleop
{

class LifecycleNodeManager final : public OrderedResourceManager
{
public:
  explicit LifecycleNodeManager(rclcpp::Node::SharedPtr node);

  bool register_lifecycle_nodes_for_ids(
    const std::vector<std::reference_wrapper<const std::vector<std::string>>> & lifecycle_node_names_for_ids);

private:
  struct Params
  {
    bool log_lifecycle_nodes = false;
    bool impatient = false;
    double reasonable_timeout = 3.0;
    int connection_retry_count = 50;
    double connection_retry_rate = 10.0;
  };

  struct ServiceClients
  {
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state;
  };

  [[nodiscard]] bool should_log_resources() const override;
  [[nodiscard]] const char * resource_kind_plural() const override;
  bool apply_resource_change(
    const std::vector<std::string> & lifecycle_nodes_to_deactivate,
    const std::vector<std::string> & lifecycle_nodes_to_activate) override;
  RefreshResult refresh_current_active_resources() override;

  std::optional<lifecycle_msgs::msg::State> get_state_for_node(const std::string & node_name);
  bool ensure_node_active(const std::string & node_name);
  bool ensure_node_inactive(const std::string & node_name);
  bool request_transition(const std::string & node_name, uint8_t transition_id, const char * transition_name);

  Params params_{};
  std::map<std::string, ServiceClients> clients_by_node_name_{};
};

}  // namespace teleop

#endif  // TELEOP_CORE__CONTROL_MODES__LIFECYCLE_NODE_MANAGER_HPP_
