// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "teleop_core/control_modes/control_mode_manager_state_publisher.hpp"
#include "teleop_core/control_modes/control_mode_manager.hpp"

namespace teleop::internal
{

namespace
{
using control_mode::ControlMode;
}  // namespace

void ControlModeManagerStatePublisher::LockedListener::on_event_invoked(const rclcpp::Time& now)
{
  // This is run whenever the down or up event is invoked
  parent.publish_state(now);
}

ControlModeManagerStatePublisher::ControlModeManagerStatePublisher(
  std::shared_ptr<rclcpp::Node> node,
  ControlModeManager& manager)
    : node_(node),
      manager_(manager)
{
  const auto logger = node_->get_logger();

  // Get parameters
  params_ = Params();
  auto default_state_topic = std::string(node_->get_name()) + "/locked";
  node->get_parameter_or<std::string>("locked_topic", params_.state_topic, default_state_topic);

  auto default_control_mode_topic = std::string(node_->get_name()) + "/active_control_modes";
  node->get_parameter_or<std::string>("active_control_modes_topic", params_.control_mode_state_topic, default_control_mode_topic);

  // QoS settings
  rclcpp::QoS qos(1);
  qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  qos.reliable();

  // Create publisher
  state_publisher_ = node->create_publisher<teleop_msgs::msg::TeleopState>(params_.state_topic, qos);
  control_mode_publisher_ = node->create_publisher<teleop_msgs::msg::ControlModeState>(params_.control_mode_state_topic, qos);
}
ControlModeManagerStatePublisher::~ControlModeManagerStatePublisher()
{
  auto now = node_->now();

  // Publish inactive message when shutting down.
  auto msg = std::make_unique<teleop_msgs::msg::TeleopState>();
  msg->header.stamp = now;
  msg->active = false;
  state_publisher_->publish(std::move(msg));

  // publish input names and active status
  auto msg_cm = std::make_unique<teleop_msgs::msg::ControlModeState>();
  msg_cm->header.stamp = now;
  msg_cm->names = control_mode_names_;
  msg_cm->active.reserve(manager_.size());

  for (auto & [name, control_mode] : manager_)
  {
    msg_cm->active.emplace_back(false);
  }
  control_mode_publisher_->publish(std::move(msg_cm));
}

void ControlModeManagerStatePublisher::configure_inputs(control_mode::Inputs inputs)
{
  locked_ = inputs.buttons["locked"];

  inputs.events["locked/down"]->subscribe(down_listener_);
  inputs.events["locked/up"]->subscribe(up_listener_);
}

void ControlModeManagerStatePublisher::publish_state(const rclcpp::Time& now)
{
  auto msg = std::make_unique<teleop_msgs::msg::TeleopState>();
  msg->header.stamp = now;
  msg->active = true;
  msg->locked = locked_.value();

  state_publisher_->publish(std::move(msg));
}

void ControlModeManagerStatePublisher::publish_control_mode_states(const rclcpp::Time& now)
{
  // Record names first time publishing.
  if (control_mode_names_.empty())
  {
    control_mode_names_.reserve(manager_.size());

    for (auto & [name, control_mode] : manager_)
    {
      control_mode_names_.emplace_back(name);
    }
  }

  // publish input names and active status
  auto msg = std::make_unique<teleop_msgs::msg::ControlModeState>();
  msg->header.stamp = now;
  msg->names = control_mode_names_;
  msg->active.reserve(manager_.size());

  for (auto & [name, control_mode] : manager_)
  {
    msg->active.emplace_back(control_mode->is_active());
  }

  control_mode_publisher_->publish(std::move(msg));
}

}  // namespace teleop::internal
