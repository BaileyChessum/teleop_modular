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
#ifndef INPUT_PUBLISHER_MODE__INPUT_PUBLISHER_MODE_HPP_
#define INPUT_PUBLISHER_MODE__INPUT_PUBLISHER_MODE_HPP_

#include <rclcpp/time.hpp>
#include <string>
#include "control_mode/control_mode.hpp"
#include "input_publisher_mode/visibility_control.h"
#include "teleop_msgs/teleop_msgs/msg/input_names.hpp"
#include "teleop_msgs/teleop_msgs/msg/combined_input_values.hpp"

namespace input_publisher_mode
{

using namespace control_mode;

/**
 * \class a generic control mode that forwards inputs from teleop_modular.
 */
class INPUT_PUBLISHER_MODE_PUBLIC InputPublisherMode : public ControlMode
{
public:
  InputPublisherMode();

  return_type on_init() override;
  CallbackReturn on_configure(const State & previous_state) override;
  void on_configure_inputs(Inputs inputs) override;

  /**
   * \brief Publishes a message to tell the control system to do nothing. Used when the control mode is locked, and
   * called once when deactivated.
   *
   * \param[in] now The time to associate with the 'halt' message.
   */
  void publish_halt_message(const rclcpp::Time & now) const;

  /**
   * \brief Publishes a message containing the axis and button names.
   */
  void publish_input_names_message() const;

  CallbackReturn on_activate(const State & previous_state) override;
  return_type on_update(const rclcpp::Time & now, const rclcpp::Duration & period) override;

  CallbackReturn on_deactivate(const State & previous_state) override;
  CallbackReturn on_cleanup(const State & previous_state) override;
  CallbackReturn on_error(const State & previous_state) override;
  CallbackReturn on_shutdown(const State & previous_state) override;

protected:
  ~InputPublisherMode() override;

private:
  /// Helper struct to hold parameters used by the control mode.
  struct Params {
    /// The topic name to send input name messages to.
    std::string input_names_topic = "";
    /// The topic name to send input messages to.
    std::string inputs_topic = "";
    /// The ROS2 topic Quality of Service value to use in inputs_publisher_.
    int inputs_qos = 10;
    /// list of axis names to publish
    std::vector<std::string> axis_names = {};
    /// list of button names to publish
    std::vector<std::string> button_names = {};
  };

  /// Stores current parameter values
  Params params_;

  /// Publisher for input names
  rclcpp::Publisher<teleop_msgs::msg::InputNames>::SharedPtr names_publisher_;
  /// Publisher for inputs
  rclcpp::Publisher<teleop_msgs::msg::CombinedInputValues>::SharedPtr inputs_publisher_;

  /// Names of the buttons to publish
  std::vector<std::string> button_names_;
  /// Names of the axes to publish
  std::vector<std::string> axis_names_;
  /// Reference to the axes inputs
  std::vector<Axis::SharedPtr> axes_;
  /// reference to the button inputs
  std::vector<Button::SharedPtr> buttons_;

  /// Count of the number of axes
  std::uint16_t axis_count_;
  /// Count of the number of buttons
  std::uint16_t button_count_;

  /// Static vector used to publish axis values
  std::vector<float> axis_values_;
  /// Static vector used to publish button values
  std::vector<uint8_t> button_values_;
};

}  // namespace input_publisher_mode

#endif  // INPUT_PUBLISHER_MODE__INPUT_PUBLISHER_MODE_HPP_
