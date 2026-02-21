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
// Created by nova on 21/2/26.
//

#ifndef TELEOP_MODULAR_CONTROLMODEMANAGER_STATE_PUBLISHER_HPP
#define TELEOP_MODULAR_CONTROLMODEMANAGER_STATE_PUBLISHER_HPP

#include <string>
#include <rclcpp/node.hpp>
#include "control_mode/control_mode.hpp"
#include <rclcpp/rclcpp/publisher.hpp>
#include "teleop_msgs/teleop_msgs/msg/control_mode_state.hpp"
#include "teleop_msgs/teleop_msgs/msg/teleop_state.hpp"

namespace teleop::internal
{

class ControlModeManager;

/**
 * Class responsible for publishing the current state of control modes.
 */
class ControlModeManagerStatePublisher final
{
public:
  class LockedListener : public control_mode::EventListener {
  public:
    bool is_down = false;
    ControlModeManagerStatePublisher& parent;

    LockedListener(bool is_down, ControlModeManagerStatePublisher& parent) : is_down(is_down), parent(parent) {}

    void on_event_invoked(const rclcpp::Time& now) override;
  };

  explicit ControlModeManagerStatePublisher(
    std::shared_ptr<rclcpp::Node> node,
    ControlModeManager & manager);

  ~ControlModeManagerStatePublisher();

  /**
   * \brief Exposes inputs to the control mode so that it can capture shared pointers to any inputs it needs. Called
   * after configure.
   *
   * \param inputs References to the collection of buttons and axes to be used by the control_mode.
   */
  void configure_inputs(control_mode::Inputs inputs);

  void publish_state(const rclcpp::Time& now);
  void publish_control_mode_states(const rclcpp::Time& now);

private:
  /// Helper struct to hold parameters used by the control mode.
  struct Params {
    /// The topic name to send TeleopState messages to.
    std::string state_topic = "";

    /// The topic name to send ControlModeState messages to.
    std::string control_mode_state_topic = "";
  };

  /// Stores current parameter values
  Params params_;

  /// Parent
  ControlModeManager & manager_;

  /// The owning teleop_modular ROS2 node.
  std::shared_ptr<rclcpp::Node> node_;
  /// Locked button reference
  control_mode::Button locked_;

  std::shared_ptr<LockedListener> up_listener_{std::make_shared<LockedListener>(false, *this)};
  std::shared_ptr<LockedListener> down_listener_{std::make_shared<LockedListener>(true, *this)};

  /// Publisher for inputs
  rclcpp::Publisher<teleop_msgs::msg::TeleopState>::SharedPtr state_publisher_;
  rclcpp::Publisher<teleop_msgs::msg::ControlModeState>::SharedPtr control_mode_publisher_;

  /// List of control mode names
  std::vector<std::string> control_mode_names_;
};
} // namespace teleop::internal

#endif  // TELEOP_MODULAR_CONTROLMODEMANAGER_STATE_PUBLISHER_HPP
