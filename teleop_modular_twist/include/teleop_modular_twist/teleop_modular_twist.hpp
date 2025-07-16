#ifndef TELEOP_MODULAR_TWIST__TELEOP_MODULAR_TWIST_HPP_
#define TELEOP_MODULAR_TWIST__TELEOP_MODULAR_TWIST_HPP_

#include <rclcpp/time.hpp>
#include "teleop_modular_twist/visibility_control.h"
#include "teleop_modular/control_modes/ControlMode.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "teleop_modular/inputs/Button.hpp"
#include "teleop_modular/inputs/Axis.hpp"
#include "twist_control_mode_parameters.hpp"

namespace teleop_modular_twist
{
  using namespace teleop;
  using namespace teleop::control_mode;

  class TwistControlMode : public teleop::control_mode::ControlMode
{
public:
  TwistControlMode();

  void publish_halt_message(const rclcpp::Time& now) const;

  CallbackReturn on_init() override;
  void capture_inputs(InputManager& inputs) override;
  return_type update(const rclcpp::Time &now, const rclcpp::Duration &period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

protected:
  ~TwistControlMode() override;

private:
  /// Helper function to get the euclidean length of a vector, used for normalized limits.
  double norm(double x, double y, double z);

  /// Tracks parameters
  std::shared_ptr<teleop_modular_twist::ParamListener> param_listener_{};
  teleop_modular_twist::Params params_{};

  /// Input from 0 to 1 that directly scales the output speed.
  Axis::SharedPtr speed_coefficient_;
  /// Do nothing when this is true
  Button::SharedPtr locked_;

  // Inputs for all the twist components
  Axis::SharedPtr x_;
  Axis::SharedPtr y_;
  Axis::SharedPtr z_;
  Axis::SharedPtr yaw_;
  Axis::SharedPtr pitch_;
  Axis::SharedPtr roll_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

}  // namespace teleop_modular_twist

#endif  // TELEOP_MODULAR_TWIST__TELEOP_MODULAR_TWIST_HPP_
