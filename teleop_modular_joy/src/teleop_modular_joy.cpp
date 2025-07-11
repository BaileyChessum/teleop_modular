//
// Created by nova on 6/11/25.
//

#include "teleop_modular_joy/teleop_modular_joy.hpp"
#include "teleop_modular/colors.hpp"

namespace teleop_modular_joy
{

void JoyInputSource::on_initialize()
{
  param_listener_ = std::make_shared<teleop_modular_joy::ParamListener>(get_node());
  params_ = param_listener_->get_params();

  subscription_ = get_node()->create_subscription<sensor_msgs::msg::Joy>(
      params_.topic, rclcpp::QoS(10), std::bind(&JoyInputSource::joy_callback, this, std::placeholders::_1));
}

void JoyInputSource::export_buttons(InputDeclarationList<uint8_t>& declarations)
{
  const auto logger = get_node()->get_logger();
  RCLCPP_DEBUG(logger, "Registered Buttons:");

  declarations.reserve(params_.button_definitions.size());
  for (auto& name : params_.button_definitions)
  {
    RCLCPP_DEBUG(logger, "  %s", name.c_str());
    declarations.add(name, false);
  }
}

void JoyInputSource::export_axes(InputDeclarationList<float>& declarations)
{
  const auto logger = get_node()->get_logger();
  RCLCPP_DEBUG(logger, "Registered Axes:");

  declarations.reserve(params_.axis_definitions.size());
  for (auto& name : params_.axis_definitions) {
    RCLCPP_DEBUG(logger, "  %s", name.c_str());
    declarations.add(name, 0.0);
  }
}

void JoyInputSource::joy_callback(sensor_msgs::msg::Joy::SharedPtr msg)
{
  std::unique_lock lock{ joy_msg_mutex_ };
  if (request_update(msg->header.stamp))
  {
    joy_msg_ = msg;
  }
}

void JoyInputSource::on_update(const rclcpp::Time& now, InputValueSpans values)
{
  sensor_msgs::msg::Joy::SharedPtr joy_msg;

  {  // Copy the pointer with mutex ownership
    std::unique_lock lock{ joy_msg_mutex_ };
    joy_msg = joy_msg_;
  }

  if (!joy_msg)
    return;

  const auto logger = get_node()->get_logger();

  // Copy values from joy_msg->buttons to values.buttons one by one
  size_t button_count = std::min(joy_msg->buttons.size(), values.buttons.size());
  for (size_t i = 0; i < button_count; i++)
  {
    values.buttons[i] = static_cast<uint8_t>(joy_msg->buttons[i] != 0);
  }

  // Copy values from joy_msg->axes to values.axes using std::copy
  size_t axis_count = std::min(joy_msg->axes.size(), values.axes.size());
  std::copy(joy_msg->axes.begin(), joy_msg->axes.begin() + static_cast<std::ptrdiff_t>(axis_count), values.axes.begin());
}

}  // namespace teleop_modular

#include <pluginlib/class_list_macros.hpp>

CLASS_LOADER_REGISTER_CLASS(teleop_modular_joy::JoyInputSource, teleop_modular::InputSource);
