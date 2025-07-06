//
// Created by nova on 6/11/25.
//

#include "../../../include/teleop/input_sources/joy/JoyInputSource.hpp"

#include "colors.hpp"

namespace teleop
{

void JoyInputSource::on_initialize()
{
  param_listener_ = std::make_shared<joy_input_source::ParamListener>(node_);
  params_ = param_listener_->get_params();

  subscription_ = get_node()->create_subscription<sensor_msgs::msg::Joy>(
      params_.topic, rclcpp::QoS(10), std::bind(&JoyInputSource::joy_callback, this, std::placeholders::_1));
}

void JoyInputSource::export_buttons(std::vector<InputDeclaration<bool>>& declarations)
{
  const auto logger = get_node()->get_logger();
  RCLCPP_DEBUG(logger, "Registered Buttons:");

  buttons_.clear();
  for (auto& name : params_.button_definitions)
  {
    RCLCPP_DEBUG(logger, "  %s", name.c_str());

    auto& button = buttons_.emplace_back(name, false);
    //    declarations.emplace_back(button.name, button.value);
    declarations.emplace_back(button.name, std::ref(button.value));
  }
}

void JoyInputSource::export_button_names(std::vector<std::string>& declarations)
{
  declarations = params_.button_definitions;
}

void JoyInputSource::export_axes(std::vector<InputDeclaration<double>>& declarations)
{
  const auto logger = get_node()->get_logger();
  RCLCPP_DEBUG(logger, "Registered Axes:");

  axes_.clear();
  for (auto& name : params_.axis_definitions)
  {
    RCLCPP_DEBUG(logger, "  %s", name.c_str());

    auto& axis = axes_.emplace_back(name, 0.0);
    //    declarations.emplace_back(axis.name, axis.value);
    declarations.emplace_back(axis);
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

void JoyInputSource::on_update(const rclcpp::Time& now)
{
  sensor_msgs::msg::Joy::SharedPtr joy_msg;

  {  // Copy the pointer with mutex ownership
    std::unique_lock lock{ joy_msg_mutex_ };
    joy_msg = joy_msg_;
  }

  if (!joy_msg)
    return;

  const auto logger = get_node()->get_logger();

  RCLCPP_DEBUG(logger, "Updating Buttons:");
  size_t button_count = std::min(joy_msg->buttons.size(), buttons_.size());
  for (size_t i = 0; i < button_count; i++)
  {
    buttons_[i].value = joy_msg->buttons[i];
    RCLCPP_DEBUG(logger, "  [%d] %s \t%s <- %s", i, buttons_[i].name.c_str(), buttons_[i].value ? "true" : "false",
                 joy_msg->buttons[i] ? "true" : "false");
  }

  size_t axis_count = std::min(joy_msg->axes.size(), axes_.size());
  for (size_t i = 0; i < axis_count; i++)
  {
    axes_[i].value = joy_msg->axes[i];
    RCLCPP_DEBUG(logger, "  [%d] %s \t%f <- %f", i, axes_[i].name.c_str(), axes_[i].value, joy_msg->axes[i]);
  }
}

}  // namespace teleop

#include <pluginlib/class_list_macros.hpp>

CLASS_LOADER_REGISTER_CLASS(teleop::JoyInputSource, teleop::InputSource);
