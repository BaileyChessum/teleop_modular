// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Created by Felicity Matthews on 31/8/25.
//
#include "input_publisher_mode/input_publisher_mode.hpp"
#include "input_publisher_mode/colors.hpp"
#include <sstream>

namespace input_publisher_mode
{

InputPublisherMode::InputPublisherMode() = default;

InputPublisherMode::~InputPublisherMode() = default;

return_type InputPublisherMode::on_init()
{
  auto node = get_node();

  // Do any initialization logic here!
  // This effectively replaces the constructor for anything that depends on get_node()

  // Declare parameters here! Or consider using something like generate_parameter_library instead.
  node->declare_parameter<std::string>("input_names_topic", "");
  node->declare_parameter<std::string>("inputs_topic", "");
  node->declare_parameter<int>("inputs_qos", 10);
  node->declare_parameter<std::vector<std::string>>("axis_names", {});
  node->declare_parameter<std::vector<std::string>>("button_names", {});

  return return_type::OK;
}

CallbackReturn InputPublisherMode::on_configure(const State &)
{
  auto node = get_node();
  const auto logger = get_node()->get_logger();

  // Use this callback method to get any parameters for your control mode!
  params_ = Params();
  node->get_parameter<std::string>("input_names_topic", params_.input_names_topic);
  node->get_parameter<std::string>("inputs_topic", params_.inputs_topic);
  node->get_parameter<int>("inputs_qos", params_.inputs_qos);
  node->get_parameter<std::vector<std::string>>("axis_names", params_.axis_names);
  node->get_parameter<std::vector<std::string>>("button_names", params_.button_names);

  // Create the publishers based on the params we just got
  if (params_.input_names_topic.empty()) {
    // You've probably made a mistake if the topic isn't set!
    RCLCPP_ERROR(logger, "The \"input_names_topic\" parameter must be set to a valid topic name!");
    return CallbackReturn::ERROR;
  }
  if (params_.inputs_topic.empty()) {
    params_.inputs_topic = params_.input_names_topic + "/values";
  }

  // QoS settings
  rclcpp::QoS qos(10);
  qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  // Create publishers
  names_publisher_ = get_node()->create_publisher<teleop_msgs::msg::InputNames>(params_.input_names_topic, qos);
  inputs_publisher_ = get_node()->create_publisher<teleop_msgs::msg::CombinedInputValues>(params_.inputs_topic, params_.inputs_qos);

  // Initialise button and axis names
  if (params_.axis_names.empty() && params_.button_names.empty())
  {
    RCLCPP_ERROR(logger, "One of \"axis_names\" or \"button_names\" parameters must be non-empty!");
    return CallbackReturn::ERROR;
  }

  axis_names_ = params_.axis_names;
  button_names_ = params_.button_names;

  // Print axes and button names prettily
  std::stringstream log;

  log << "Publishing inputs:";
  log << "\n  " C_QUIET << names_publisher_->get_topic_name() << "        \t[teleop_msgs/msg/InputNames]" C_RESET;
  log << "\n  " C_QUIET<< inputs_publisher_->get_topic_name() << " \t[teleop_msgs/msg/CombinedInputValues]" C_RESET;

  log << "\n\tButtons:";
  for (auto& button : button_names_)
  {
    log << "\n\t  - " C_INPUT << button << C_RESET;
  }

  log << "\n\tAxes:";
  for (auto& axis : axis_names_)
  {
    log << "\n\t  - " C_INPUT << axis << C_RESET;
  }
  RCLCPP_INFO(logger, "%s\n", log.str().c_str());

  return CallbackReturn::SUCCESS;
}

void InputPublisherMode::on_configure_inputs(Inputs inputs)
{
  // This method is always run after on_configure(),
  // so you can assume that you already have any necessary parameters

  // This method is called twice so clear inputs and values vectors
  axes_.clear();
  buttons_.clear();
  axis_values_.clear();
  button_values_.clear();

  // Capture inputs
  for (const auto & axis_name : axis_names_)
  {
    axes_.push_back(inputs.axes[axis_name]);
    axis_values_.push_back(0);
  }
  for (const auto & button_name : button_names_)
  {
    buttons_.push_back(inputs.buttons[button_name]);
    button_values_.push_back(0);
  }
}

CallbackReturn InputPublisherMode::on_activate(const State &)
{
  // Publish input names
  publish_input_names_message();

  return CallbackReturn::SUCCESS;
}

CallbackReturn InputPublisherMode::on_deactivate(const State &)
{
  publish_halt_message(get_node()->now());
  return CallbackReturn::SUCCESS;
}

void InputPublisherMode::publish_input_names_message() const
{
  const auto logger = get_node()->get_logger();

  // publish input names
  auto msg = std::make_unique<teleop_msgs::msg::InputNames>();
  msg->value_topic = names_publisher_->get_topic_name();
  msg->axis_names = axis_names_;
  msg->button_names = button_names_;
  names_publisher_->publish(std::move(msg));
}

void InputPublisherMode::publish_halt_message(const rclcpp::Time & now) const
{
  // TODO: Implement for your message type, or remove the method if it is not appropriate for the use case.
  auto msg = std::make_unique<teleop_msgs::msg::CombinedInputValues>();
  msg->values.header.stamp = now;
  msg->events.header.stamp = now;
  inputs_publisher_->publish(std::move(msg));
}

return_type InputPublisherMode::on_update(const rclcpp::Time & now, const rclcpp::Duration & period)
{
  const auto logger = get_node()->get_logger();

  // Don't move when locked
  if (is_locked()) {
    publish_halt_message(now);
    return return_type::OK;
  }

  // Get input values either with input_->value() or by referencing and implicitly casting *input_
  // const float speed = std::max(speed_->value(), 0.0f);

  // Construct and send a message using values from inputs
  auto msg = std::make_unique<teleop_msgs::msg::CombinedInputValues>();
  msg->values.header.stamp = now;
  msg->events.header.stamp = now;

  // Update values
  for (int i = 0; i < static_cast<int>(axis_names_.size()); i++)
  {
    axis_values_[i] = axes_[i].value();
  }
  for (int i = 0; i < static_cast<int>(button_names_.size()); i++)
  {
    button_values_[i] = buttons_[i].value();
  }

  msg->values.axes = axis_values_;
  msg->values.buttons = button_values_;

  inputs_publisher_->publish(std::move(msg));

  return return_type::OK;
}

CallbackReturn InputPublisherMode::on_error(const State &)
{
  // Called when any callback function returns CallbackReturn::ERROR

  return CallbackReturn::SUCCESS;
}

CallbackReturn InputPublisherMode::on_cleanup(const State &)
{
  // Clear all state and return the control mode to a functionally equivalent state as after on_init() was first called.

  return CallbackReturn::SUCCESS;
}

CallbackReturn InputPublisherMode::on_shutdown(const State &)
{
  // Clean up anything from on_init()

  return CallbackReturn::SUCCESS;
}

}  // namespace input_publisher_mode

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(input_publisher_mode::InputPublisherMode, control_mode::ControlMode);
