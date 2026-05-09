// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#ifndef TELEOP_CORE__CONTROL_MODES__CONTROLLER_MANAGER_MANAGER_HPP_
#define TELEOP_CORE__CONTROL_MODES__CONTROLLER_MANAGER_MANAGER_HPP_

#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/node.hpp>

#include "teleop_core/control_modes/ordered_resource_manager.hpp"

namespace teleop
{

class ControllerManagerManager final : public OrderedResourceManager
{
public:
  explicit ControllerManagerManager(rclcpp::Node::SharedPtr node);

  bool register_controllers_for_ids(
    const std::vector<std::reference_wrapper<const std::vector<std::string>>> & controller_names_for_ids);

private:
  struct Params
  {
    std::string controller_manager = "/controller_manager";
    bool log_controllers = false;
    bool impatient = false;
    int strictness = 2;
    bool activate_asap = true;
    builtin_interfaces::msg::Duration timeout{};
    double reasonable_timeout = 3.0;
    int connection_retry_count = 50;
    double connection_retry_rate = 10.0;
  };

  [[nodiscard]] bool should_log_resources() const override;
  [[nodiscard]] const char * resource_kind_plural() const override;
  bool apply_resource_change(
    const std::vector<std::string> & controllers_to_deactivate,
    const std::vector<std::string> & controllers_to_activate) override;
  RefreshResult refresh_current_active_resources() override;

  Params params_{};
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
};

}  // namespace teleop

#endif  // TELEOP_CORE__CONTROL_MODES__CONTROLLER_MANAGER_MANAGER_HPP_
