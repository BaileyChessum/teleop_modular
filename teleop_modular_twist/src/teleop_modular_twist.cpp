#include "teleop_modular_twist/teleop_modular_twist.hpp"

namespace teleop_modular_twist
{

using namespace teleop::control_mode;
using teleop::control_mode::ControlMode;

TwistControlMode::TwistControlMode() = default;

TwistControlMode::~TwistControlMode() = default;

CallbackReturn TwistControlMode::on_init()
{
  param_listener_ = std::make_shared<teleop_modular_twist::ParamListener>(node_);
  params_ = param_listener_->get_params();
}

CallbackReturn TwistControlMode::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  const auto logger = get_node()->get_logger();

  if (param_listener_->is_old(params_))
    params_ = param_listener_->get_params();

  // Create publisher
  const rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
                                      .best_effort()
                                      .transient_local()
                                      .keep_last(1);
  publisher_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(params_.topic, qos_profile);
}

void TwistControlMode::capture_inputs(InputManager& inputs)
{
  auto& axes = inputs.get_axes();
  auto& buttons = inputs.get_buttons();

  speed_coefficient_ = axes[params_.input_names.speed];
  locked_ = buttons[params_.input_names.locked];

  x_ = axes[params_.input_names.twist_x];
  y_ = axes[params_.input_names.twist_y];
  z_ = axes[params_.input_names.twist_z];
  roll_ = axes[params_.input_names.twist_roll];
  pitch_ = axes[params_.input_names.twist_pitch];
  yaw_ = axes[params_.input_names.twist_yaw];
}

CallbackReturn TwistControlMode::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn TwistControlMode::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

void TwistControlMode::publish_halt_message(const rclcpp::Time& now) const
{
  auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  msg->header.stamp = now;
  publisher_->publish(std::move(msg));
}

return_type TwistControlMode::update(const rclcpp::Time& now, const rclcpp::Duration& period)
{
  auto logger = get_node()->get_logger();

  // Don't move when locked
  if (*locked_)
  {
    publish_halt_message(now);
    return return_type::OK;
  }

  const float speed_coefficient = std::clamp(speed_coefficient_->value(), 0.0f, 1.0f);
  auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  msg->header.stamp = now;

  msg->twist.linear.x = *x_ * speed_coefficient * params_.max_speed.linear;
  msg->twist.linear.y = y_->value() * speed_coefficient * params_.max_speed.linear;
  msg->twist.linear.z = z_->value() * speed_coefficient * params_.max_speed.linear;
  msg->twist.angular.x = roll_->value() * speed_coefficient * params_.max_speed.angular;
  msg->twist.angular.y = pitch_->value() * speed_coefficient * params_.max_speed.angular;
  msg->twist.angular.z = yaw_->value() * speed_coefficient * params_.max_speed.angular;

  if (params_.max_speed.normalized)
  {
    auto linear_input_norm = norm(x_->value(), y_->value(), z_->value());
    if (linear_input_norm > 1.0)
    {
      msg->twist.linear.x /= linear_input_norm;
      msg->twist.linear.y /= linear_input_norm;
      msg->twist.linear.z /= linear_input_norm;
    }

    auto angular_input_norm = norm(x_->value(), y_->value(), z_->value());
    if (angular_input_norm > 1.0)
    {
      msg->twist.angular.x /= angular_input_norm;
      msg->twist.angular.y /= angular_input_norm;
      msg->twist.angular.z /= angular_input_norm;
    }
  }

  msg->header.frame_id = params_.frame_id;

  publisher_->publish(std::move(msg));

  return return_type::OK;
}

double TwistControlMode::norm(double x, double y, double z)
{
  return std::sqrt(x * x + y * y + z * z);
}

CallbackReturn TwistControlMode::on_error(const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn TwistControlMode::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn TwistControlMode::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

}  // namespace teleop_modular_twist

#include <pluginlib/class_list_macros.hpp>

CLASS_LOADER_REGISTER_CLASS(teleop_modular_twist::TwistControlMode, teleop::control_mode::ControlMode);