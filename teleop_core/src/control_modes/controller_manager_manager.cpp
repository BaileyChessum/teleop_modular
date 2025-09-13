// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
//
// Created by Bailey Chessum on 13/9/2025.
//

#include "teleop_core/control_modes/controller_manager_manager.hpp"

namespace teleop
{

ControllerManagerManager::Context::Context(const rclcpp::Node::SharedPtr& node)
    : logger_(node->get_logger().get_child("controller_manager_manager")) {
  // Create service clients
  // TODO: Allow for controller_manager to be specified like with -c args. This would be needed to support multiple
  //  controller manager manager instances
  switch_controller_client_ =
      node->create_client<controller_manager_msgs::srv::SwitchController>(
          "/controller_manager/switch_controller");
}

}  // namespace teleop