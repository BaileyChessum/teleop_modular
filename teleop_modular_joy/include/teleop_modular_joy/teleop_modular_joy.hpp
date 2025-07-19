// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Created by nova on 6/11/25.
//

#ifndef TELEOP_MODULAR_JOYINPUTSOURCE_HPP
#define TELEOP_MODULAR_JOYINPUTSOURCE_HPP

#include <sensor_msgs/msg/joy.hpp>
#include <utility>
#include "input_source/input_source.hpp"
#include "joy_input_source_parameters.hpp"

namespace teleop_modular_joy
{
using namespace input_source;

class JoyInputSource final : public InputSource
{
protected:
  return_type on_init() override;
  return_type on_update(const rclcpp::Time & now, InputValueSpans values) override;

  void export_buttons(InputDeclarationList<uint8_t> & declarations) override;
  void export_axes(InputDeclarationList<float> & declarations) override;

private:
  void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg);

  std::shared_ptr<teleop_modular_joy::ParamListener> param_listener_;
  teleop_modular_joy::Params params_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  sensor_msgs::msg::Joy::SharedPtr joy_msg_ = nullptr;
  std::mutex joy_msg_mutex_{};
};

}  // namespace teleop_modular_joy

#endif  // TELEOP_MODULAR_JOYINPUTSOURCE_HPP
