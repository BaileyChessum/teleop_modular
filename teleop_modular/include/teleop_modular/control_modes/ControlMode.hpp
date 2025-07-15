//
// Created by nova on 6/4/25.
//

#ifndef TELEOP_MODULAR_CONTROLMODE_HPP
#define TELEOP_MODULAR_CONTROLMODE_HPP

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/node.hpp>
// #include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "teleop_modular/inputs/InputCommon.hpp"
#include "teleop_modular/inputs/InputManager.hpp"

namespace teleop::control_mode
{


/// To replace returning bools to indicate whether an operation was successful, or failed due to some error. Used to
/// be more explicit.
enum class return_type : bool
{
  OK = false,
  ERROR = true,
};

/// These usings are added with hopes to reduce the length of type names for implementers of control modes
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using teleop_modular::InputManager;

/**
 * Base class for a control mode used in teleoperation
 */
class ControlMode : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ~ControlMode() override;
  ControlMode() = default;

  /// This function effectively replaces the initializer
  return_type init(const std::string& name, const std::string& node_namespace,
                      const rclcpp::NodeOptions& node_options);

  // Accessors
  /// Name of the control mode, which the control mode is indexed by
  [[nodiscard]] const std::string& get_name() const
  {
    return name_;
  }
  [[nodiscard]] const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& get_node() const
  {
    return node_;
  }

  virtual void capture_inputs(InputManager& inputs) = 0;
  virtual return_type update(const rclcpp::Time& now, const rclcpp::Duration& period) = 0;

  [[nodiscard]] const rclcpp_lifecycle::State& get_lifecycle_state() const;

protected:
  virtual CallbackReturn on_init() = 0;

  /// The ROS2 node created by teleop_modular, which we get params from (for base and child classes)
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

private:
  /// Name of the control mode, which the control mode is indexed by
  std::string name_;
};

}  // namespace teleop_modular

#endif  // TELEOP_MODULAR_CONTROLMODE_HPP
