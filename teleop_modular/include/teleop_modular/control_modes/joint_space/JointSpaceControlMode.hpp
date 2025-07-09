//
// Created by Bailey Chessum on 6/4/25.
//

#ifndef TELEOP_MODULAR_JOINTSPACECONTROLMODE_HPP
#define TELEOP_MODULAR_JOINTSPACECONTROLMODE_HPP

#include "joint_space_control_mode_parameters.hpp"
#include "teleop_modular/control_modes/ControlMode.hpp"
#include "nova_interfaces/msg/arm_fk_velocity_targets.hpp"
#include "teleop_modular/inputs/Button.hpp"
#include "teleop_modular/inputs/Axis.hpp"
// generate_parameter_library_cpp include/teleop_modular/control_modes/joint_space/joint_space_control_mode_parameters.hpp
// src/control_modes/joint_space/joint_space_control_mode_parameters.yaml

namespace teleop_modular
{
/**
 * Control mode for moving joint velocities directly
 */
class JointSpaceControlMode final : public ControlMode
{
public:
  explicit JointSpaceControlMode() = default;

  void on_initialize() override;

  void on_configure(InputManager& inputs) override;
  void on_activate() override;
  void on_deactivate() override;

  void publish_halt_message(const rclcpp::Time& now) const;

  void update(const rclcpp::Time& now, const rclcpp::Duration& period) override;

protected:
  struct JointHandle
  {
    std::string name;
    joint_space_control_mode::Params::Joints::MapJointDefinitions config;
    Axis::SharedPtr input;
  };

  ~JointSpaceControlMode() override = default;

  /// Tracks parameters
  std::shared_ptr<joint_space_control_mode::ParamListener> param_listener_;
  joint_space_control_mode::Params params_;

  /// Input from 0 to 1 that directly scales the output speed.
  Axis::SharedPtr speed_coefficient_;
  Button::SharedPtr locked_;

  /// Inputs for each joint in params_.joint_definitions, in the same order as params_.joint_definitions
  std::vector<JointHandle> joints_;

  /// Publishes messages to the arm controller
  rclcpp::Publisher<nova_interfaces::msg::ArmFkVelocityTargets>::SharedPtr publisher_;
};

}  // namespace teleop_modular

#endif  // TELEOP_MODULAR_JOINTSPACECONTROLMODE_HPP
