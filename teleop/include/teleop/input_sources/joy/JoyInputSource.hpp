//
// Created by nova on 6/11/25.
//

#ifndef TELEOP_JOYINPUTSOURCE_HPP
#define TELEOP_JOYINPUTSOURCE_HPP

#include <sensor_msgs/msg/joy.hpp>
#include <utility>
#include "teleop/input_sources/InputSource.hpp"
#include "joy_input_source_parameters.hpp"
#include "teleop/inputs/state/State.hpp"

namespace teleop
{

class JoyInputSource final : public InputSource
{
protected:
  void on_initialize() override;
  void on_update(const rclcpp::Time& now) override;

  void export_buttons(std::vector<InputDeclaration<bool>>& declarations) override;
  void export_axes(std::vector<InputDeclaration<double>>& definitions) override;

private:
  void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg);

  std::shared_ptr<joy_input_source::ParamListener> param_listener_;
  joy_input_source::Params params_;

  std::vector<std::string> button_names;
  std::vector<bool> button_names;

  std::vector<State<double>> axes_;
  std::vector<State<bool>> buttons_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  sensor_msgs::msg::Joy::SharedPtr joy_msg_ = nullptr;
  std::mutex joy_msg_mutex_{};
};

}  // namespace teleop

#endif  // TELEOP_JOYINPUTSOURCE_HPP
