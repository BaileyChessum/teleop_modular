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
#include <cmath>
#include <teleop_core/utilities/get_parameter.hpp>

namespace {
/**
 * One-off helper function to convert a double to a duration
 */
inline builtin_interfaces::msg::Duration to_duration(double seconds) {
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(std::floor(seconds));
  duration.nanosec = static_cast<uint32_t>(std::round((seconds - duration.sec) * 1e9));

  // Handle rounding overflow
  if (duration.nanosec == 1000000000) {
    duration.sec += 1;
    duration.nanosec = 0;
  }
  return duration;
}
}   // namespace

namespace teleop
{

ControllerManagerManager::Context::Context(const rclcpp::Node::SharedPtr& node)
    : node(node), logger(node->get_logger().get_child("controller_manager_manager")) {

  // Set up parameters
  auto parameters_interface = node->get_node_parameters_interface();

  params.controller_manager =
      utils::get_parameter_or_default<std::string>(
          parameters_interface,
          "controller_manager",
          "The name of the controller manager to use when switching controllers for control modes. Don't "
          "include a \'/\' at the end.",
          "/controller_manager"
      );

  params.log_controllers =
      utils::get_parameter_or_default<bool>(
          parameters_interface,
          "log_controllers",
          "Whether to log debug info about controller activations.",
          false
      );

  params.impatient =
      utils::get_parameter_or_default<bool>(
          parameters_interface,
          "controller_manager_manager.impatient",
          "Whether to wait (for the specified reasonable time) for previous requests to finish before "
          "starting new switch requests.",
          false
      );

  params.strictness =
      utils::get_parameter_or_default<int64_t>(
          parameters_interface,
          "controller_manager_manager.strictness",
          "strictness value to use in /switch_controller service messages.\n"
          "the strictness (STRICT=2, BEST_EFFORT=1, AUTO=3, or FORCE_AUTO=4). STRICT by default. See "
          "https://github.com/ros-controls/ros2_control/blob/master/controller_manager_msgs/srv/SwitchController.srv "
          "for more info.",
          2
      );

  params.activate_asap =
      utils::get_parameter_or_default<bool>(
          parameters_interface,
          "controller_manager_manager.activate_asap",
          "activate_asap value to use in /switch_controller service messages.\n"
          "Whether to activate the controllers as soon as their hardware dependencies are ready, or otherwise wait for "
          "all interfaces to be ready.",
          true
      );

  const auto timeout_seconds =
      utils::get_parameter_or_default<double>(
          parameters_interface,
          "controller_manager_manager.timeout",
          "timeout value (in seconds) to use in /switch_controller service messages.\n"
          "the timeout before aborting pending controllers. Zero for infinite",
          0.0
      );
  params.timeout = to_duration(timeout_seconds);

  params.reasonable_timeout =
      utils::get_parameter_or_default<double>(
          parameters_interface,
          "controller_manager_manager.reasonable_timeout",
          "Time in seconds to keep waiting for a previous switch request to finish before giving up on it "
          "and making a new switch request.",
          3.0
      );

  params.connection_retry_count =
      utils::get_parameter_or_default<int64_t>(
          parameters_interface,
          "controller_manager_manager.connection_retry_count",
          "Number of times to check if controller manager services are available before giving up. Set to -1 "
          "to never stop trying.",
          50
      );

  params.connection_retry_rate =
      utils::get_parameter_or_default<double>(
          parameters_interface,
          "controller_manager_manager.connection_retry_rate",
          "The rate at which service calls to the controller manager should be retried, in hz.",
          10.0
      );

  // Create service clients
  // TODO: Allow for controller_manager to be specified like with -c args. This would be needed to support multiple
  //  controller manager manager instances
  switch_controller_client =
      node->create_client<controller_manager_msgs::srv::SwitchController>(
          params.controller_manager + "/switch_controller");
  list_controllers_client =
      node->create_client<controller_manager_msgs::srv::ListControllers>(
          params.controller_manager + "/list_controllers");
}

}  // namespace teleop