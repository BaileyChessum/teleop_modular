// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#ifndef TELEOP_MODULAR_TWIST__TELEOP_MODULAR_TWIST_HPP_
#define TELEOP_MODULAR_TWIST__TELEOP_MODULAR_TWIST_HPP_

#include <rclcpp/time.hpp>
#include "teleop_modular_twist/visibility_control.h"
#include "control_mode/control_mode.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "twist_control_mode_parameters.hpp"

namespace teleop_modular_twist
{
using namespace control_mode;

class TELEOP_MODULAR_TWIST_PUBLIC TwistControlMode : public ControlMode
{
public:
  TwistControlMode();

  void publish_halt_message(const rclcpp::Time & now) const;

  return_type on_init() override;
  void capture_inputs(Inputs inputs) override;
  return_type update(const rclcpp::Time & now, const rclcpp::Duration & period) override;

  CallbackReturn on_configure(const State & previous_state) override;
  CallbackReturn on_activate(const State & previous_state) override;
  CallbackReturn on_deactivate(const State & previous_state) override;
  CallbackReturn on_cleanup(const State & previous_state) override;
  CallbackReturn on_error(const State & previous_state) override;
  CallbackReturn on_shutdown(const State & previous_state) override;

protected:
  ~TwistControlMode() override;

private:
  /// We use infinity as the default limit, as it will effectively not impose a limit while avoiding branching
  static constexpr double infinity = std::numeric_limits<double>::infinity();
  /// Helper type to hold 3 Axis::SharedPtrs to make a up a vector
  using AxisVector3 = std::array<Axis::SharedPtr, 3>;
  /// Helper type to hold 3 Axis::SharedPtrs to make a up a vector. You could also use Eigen for more complex use cases.
  using NumberVector3 = std::array<double, 3>;

  /// Helper struct to avoid duplicating code for the nearly identical logic for linear and angular components of twist.
  struct VectorHandle
  {
    /// Our actual inputs for the vector3
    AxisVector3 axes;
    /// A scale to multiply axis values by when populating vector3 messages
    NumberVector3 scale = {1.0, 1.0, 1.0};
    /// limits to apply to the vector3 message
    std::optional<NumberVector3> limit = std::nullopt;

    /// Switches limiting logic from simple component-wise limiting to more complex normalization based limits.
    bool normalized_limits = true;
    /// When true, limits will be applied to the axis inputs relative to the 'speed' input.
    bool scale_limits_with_speed = true;

    void set_limits(NumberVector3 values, double all, bool normalized);
    /**
     * \brief Calculates the value for a Vector3 message based on the input axis values, and given speed_coefficient.
     *
     * This method applies limits when limit.has_value().
     * \param msg[out]  The message to put calculated values in
     * \param speed_coefficient[out]    The value to multiply all speeds by
     */
    void apply_to(geometry_msgs::msg::Vector3 & msg, double speed_coefficient);
  };

  /// Helper function to get the euclidean length of a vector, used for normalized limits.
  static double norm(double x, double y, double z);

  /// Tracks parameters
  std::shared_ptr<teleop_modular_twist::ParamListener> param_listener_{};
  teleop_modular_twist::Params params_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  /// Inputs for all the linear twist components
  VectorHandle linear_;
  /// Inputs for all the angular twist components
  VectorHandle angular_;

  /// Input from 0 to 1 that directly scales the output speed.
  Axis::SharedPtr speed_;
  /// Do nothing when this is true
  Button::SharedPtr locked_;
};

}  // namespace teleop_modular_twist

#endif  // TELEOP_MODULAR_TWIST__TELEOP_MODULAR_TWIST_HPP_
