// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#include "teleop_modular_twist/teleop_modular_twist.hpp"

namespace
{
// This value is used to avoid normalized limits that would cause a divide by zero
constexpr double EPSILON = 1e-6;
}

namespace teleop_modular_twist
{

TwistControlMode::TwistControlMode() = default;

TwistControlMode::~TwistControlMode() = default;

return_type TwistControlMode::on_init()
{
  param_listener_ = std::make_shared<teleop_modular_twist::ParamListener>(node_);

  return return_type::OK;
}

CallbackReturn TwistControlMode::on_configure(const State &)
{
  const auto logger = get_node()->get_logger();

  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();

    // Set up handle configurations for linear and angular inputs
    linear_.set_limits(
      {params_.limit.linear.x, params_.limit.linear.y, params_.limit.linear.z},
      params_.limit.linear.all, params_.limit.linear.normalized);
    linear_.scale_limits_with_speed = params_.limit.linear.scale_with_speed;

    angular_.set_limits(
      {params_.limit.angular.x, params_.limit.angular.y, params_.limit.angular.z},
      params_.limit.angular.all, params_.limit.angular.normalized);
    linear_.scale_limits_with_speed = params_.limit.angular.scale_with_speed;
  }

  if (!params_.stamped_topic.empty()) {
    stamped_publisher_ =
      get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
      params_.stamped_topic, params_.qos);
  }
  if (!params_.topic.empty()) {
    publisher_ = get_node()->create_publisher<geometry_msgs::msg::Twist>(
      params_.topic, params_.qos);
  }

  // You've probably made a mistake if you aren't publishing anything
  if (!stamped_publisher_ && !publisher_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Both stamped_topic and topic parameters aren't set! Please set at least one.");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

void TwistControlMode::capture_inputs(Inputs inputs)
{
  // TODO: Implement a remapping functionality to avoid boilerplate parameters for names, like input source remapping
  speed_ = inputs.axes[params_.input_names.speed];
  locked_ = inputs.buttons[params_.input_names.locked];

  linear_.axes = {
    inputs.axes[params_.input_names.linear.x],
    inputs.axes[params_.input_names.linear.y],
    inputs.axes[params_.input_names.linear.z]
  };

  angular_.axes = {
    inputs.axes[params_.input_names.angular.x],
    inputs.axes[params_.input_names.angular.y],
    inputs.axes[params_.input_names.angular.z]
  };
}

CallbackReturn TwistControlMode::on_activate(const State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn TwistControlMode::on_deactivate(const State &)
{
  publish_halt_message(get_node()->now());

  return CallbackReturn::SUCCESS;
}

void TwistControlMode::publish_halt_message(const rclcpp::Time & now) const
{
  if (stamped_publisher_) {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = now;
    msg->header.frame_id = params_.frame;
    stamped_publisher_->publish(std::move(msg));
  }

  if (publisher_) {
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    publisher_->publish(std::move(msg));
  }
}

return_type TwistControlMode::update(const rclcpp::Time & now, const rclcpp::Duration &)
{
  auto logger = get_node()->get_logger();

  // Don't move when locked
  if (*locked_) {
    publish_halt_message(now);
    return return_type::OK;
  }

  const float speed_coefficient = params_.use_speed_input ? std::max(speed_->value(), 0.0f) : 1.0;
  auto twist = geometry_msgs::msg::Twist();

  linear_.apply_to(twist.linear, speed_coefficient);
  angular_.apply_to(twist.angular, speed_coefficient);

  if (stamped_publisher_) {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->twist = twist;
    msg->header.stamp = now;
    msg->header.frame_id = params_.frame;
    stamped_publisher_->publish(std::move(msg));
  }

  if (publisher_) {
    publisher_->publish(twist);
  }

  return return_type::OK;
}

double TwistControlMode::norm(double x, double y, double z)
{
  return std::sqrt(x * x + y * y + z * z);
}

CallbackReturn TwistControlMode::on_error(const State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn TwistControlMode::on_cleanup(const State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn TwistControlMode::on_shutdown(const State &)
{
  linear_.axes = {nullptr, nullptr, nullptr};
  angular_.axes = {nullptr, nullptr, nullptr};
  locked_.reset();
  speed_.reset();

  publisher_.reset();
  stamped_publisher_.reset();

  return CallbackReturn::SUCCESS;
}

void TwistControlMode::VectorHandle::set_limits(
  const NumberVector3 values, double all,
  bool normalized)
{
  normalized_limits = normalized;
  limit = std::nullopt;   // Clear old values
  bool any_limit_set = false;

  double default_limit = infinity;  // infinity acts as no limit, so default to infinity when 'all' is not set
  if (all >= 0.0) {
    default_limit = all;
    any_limit_set = true;
  }

  // Calculate the limit -- but use a separate array for now in case they are all infinity
  NumberVector3 potential_limit;
  for (size_t i = 0; i < 3; ++i) {
    // This condition is true when the limit for this x,y,z is not set
    if (values[i] < 0.0) {
      potential_limit[i] = default_limit;
      continue;
    }

    // Edge case to avoid divisions by zero (or near zero) when normalized
    // But also applied to non-normalized limits because it is effectively a potential optimization
    if (values[i] < EPSILON) {
      // A scale of 0 is equivalent to a limit of 0.
      // We don't set any_limit_set = true because we don't need to run limit code for this to work
      scale[i] = 0.0;
      potential_limit[i] = infinity;
      continue;
    }

    // Actually apply a limit in this case
    potential_limit[i] = values[i];
    any_limit_set = true;
  }

  // Only set the limit if there is a non-infinity limit
  if (any_limit_set) {
    limit = potential_limit;
  }
}

void TwistControlMode::VectorHandle::apply_to(
  geometry_msgs::msg::Vector3 & msg,
  double speed_coefficient)
{
  NumberVector3 result;

  if (!limit.has_value()) {
    // Calculate the values without applying the limit.
    for (size_t i = 0; i < 3; ++i)
      result[i] = *axes[i] * scale[i] * speed_coefficient;

    // Apply to the message and exit early.
    msg.x = result[0];
    msg.y = result[1];
    msg.z = result[2];
    return;
  }

  // Calculate the values, excluding the speed_coefficient if we should scale_limits_with_speed
  if (scale_limits_with_speed) {
    for (size_t i = 0; i < 3; ++i)
      result[i] = *axes[i] * scale[i];
  }
  else {
    for (size_t i = 0; i < 3; ++i)
      result[i] = *axes[i] * scale[i] * speed_coefficient;
  }

  // Apply limits
  if (normalized_limits) {
    // Apply by scaling the vector down depending on its magnitude relative to the limit
    NumberVector3 vector_to_norm;
    // Note: This could cause a division by zero if not careful! Make sure it can never be zero when setting params.
    for (size_t i = 0; i < 3; ++i) {
      vector_to_norm[i] = result[i] / (*limit)[i];
    }

    auto magnitude_relative_to_limit = std::apply(norm, vector_to_norm);
    if (magnitude_relative_to_limit > 1.0) {
      for (size_t i = 0; i < 3; ++i) {
        result[i] /= magnitude_relative_to_limit;
      }
    }
  } else {
    // Apply per component independently
    for (size_t i = 0; i < 3; ++i) {
      result[i] = std::clamp(result[i], -(*limit)[i], (*limit)[i]);
    }
  }

  // If speed_coefficient didn't scale limits before, scale them now
  if (scale_limits_with_speed) {
    for (size_t i = 0; i < 3; ++i)
      result[i] *= speed_coefficient;
  }

  // Apply to the message
  msg.x = result[0];
  msg.y = result[1];
  msg.z = result[2];
}

}  // namespace teleop_modular_twist

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(teleop_modular_twist::TwistControlMode, control_mode::ControlMode);
