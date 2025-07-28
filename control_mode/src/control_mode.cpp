// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#include "control_mode/control_mode.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace control_mode
{

ControlMode::~ControlMode()
{
  // Similar check to ControllerInterfaceBase
  // https://github.com/ros-controls/ros2_control/blob/7061ac41/controller_interface/src/controller_interface_base.cpp#L28-L39
  if (node_.get() && rclcpp::ok() &&
    node_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
  {
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "Calling shutdown on LifecycleNode due to destruction of ControlMode %s.",
      get_name().c_str());
    node_->shutdown();
  }
}

return_type ControlMode::init(
  const std::string & name, const std::string & node_namespace,
  const rclcpp::NodeOptions & node_options,
  const std::shared_ptr<rclcpp::Executor> & executor, const CommonParams & common_params)
{
  name_ = name;
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    name_, node_namespace, node_options,
    false);
  common_params_ = common_params;

  const auto logger = get_node()->get_logger();
  RCLCPP_DEBUG(
    logger, "Initializing ControlMode with name \"%s\" in namespace \"%s\"...", name_.c_str(),
    node_namespace.c_str());

  // Perform child class initialization
  switch (on_init()) {
    case return_type::OK:
      break;
    default:
      node_->shutdown();
      return return_type::ERROR;
  }

  RCLCPP_DEBUG(logger, "Setting up LifecycleNode callbacks for \"%s\"...", name_.c_str());

  node_->register_on_configure(std::bind(&ControlMode::on_configure, this, std::placeholders::_1));
  node_->register_on_activate(std::bind(&ControlMode::on_configure, this, std::placeholders::_1));
  node_->register_on_deactivate(
    std::bind(
      &ControlMode::on_deactivate, this,
      std::placeholders::_1));
  node_->register_on_error(std::bind(&ControlMode::on_error, this, std::placeholders::_1));
  node_->register_on_shutdown(std::bind(&ControlMode::on_shutdown, this, std::placeholders::_1));
  node_->register_on_cleanup(std::bind(&ControlMode::on_cleanup, this, std::placeholders::_1));

  RCLCPP_DEBUG(logger, "Adding LifecycleNode for \"%s\" to the executor.", name_.c_str());

  executor->add_node(node_->get_node_base_interface());

  RCLCPP_DEBUG(logger, "\"%s\" fully initialized.", name_.c_str());

  return return_type::OK;
}

void ControlMode::capture_inputs(Inputs inputs) {
  locked_ = inputs.buttons["locked"];
  on_capture_inputs(inputs);
}

const rclcpp_lifecycle::State & ControlMode::get_lifecycle_state() const
{
  if (!node_.get()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(get_name()),
      "Lifecycle node accessed without being initialized!");
    throw std::runtime_error("Lifecycle node accessed without being initialized!");
  }

  return node_->get_current_state();
}

bool ControlMode::is_active() const
{
  return get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

const std::vector<std::string> & ControlMode::get_controllers() const
{
  return common_params_.controllers;
}

}  // namespace control_mode
