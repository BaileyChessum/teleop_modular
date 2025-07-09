//
// Created by nova on 6/4/25.
//

#ifndef TELEOP_MODULAR_CONTROLMODE_HPP
#define TELEOP_MODULAR_CONTROLMODE_HPP

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>

#include "teleop_modular/inputs/InputCommon.hpp"
#include "teleop_modular/inputs/InputManager.hpp"

namespace teleop_modular
{
/**
 * Base class for a control mode used in teleop_modulareration
 */
class ControlMode
{
public:
  virtual ~ControlMode() = default;

  enum State
  {
    INACTIVE = 0,
    CONFIGURING = 1,
    ACTIVE = 2
  };

  /**
   * Params common to all ControlModes
   */
  struct Params
  {
    std::vector<std::string> controllers{};
  };

  ControlMode() = default;

  /// This function effectively replaces the initializer
  void initialize(const std::shared_ptr<rclcpp::Node>& node, const std::string& name);

  // Lifecycle methods
  void configure(InputManager& inputs);
  void activate();
  void deactivate();

  virtual void update(const rclcpp::Time& now, const rclcpp::Duration& period) {};

  // Accessors
  /// Name of the control mode, which the control mode is indexed by
  [[nodiscard]] const std::string& get_name() const
  {
    return name_;
  }
  /// Params, from the base ControlMode type, populated by the ControlMode base class.
  [[nodiscard]] const Params& get_base_params() const
  {
    return base_params_;
  }
  const std::shared_ptr<rclcpp::Node>& get_node() const
  {
    return node_;
  }

protected:
  virtual void on_initialize() {};

  // Lifecycle methods
  virtual void
  on_configure(InputManager& inputs /* TODO: Give node, params, and any necessary context to set itself up */) {};
  virtual void on_activate() {};
  virtual void on_deactivate() {};

  /// The ROS2 node created by teleop_modular, which we get params from (for base and child classes)
  std::shared_ptr<rclcpp::Node> node_ = nullptr;

  /// Params, from the base ControlMode type, populated by the ControlMode base class.
  Params base_params_;

private:
  /// Name of the control mode, which the control mode is indexed by
  std::string name_;
};

}  // namespace teleop_modular

#endif  // TELEOP_MODULAR_CONTROLMODE_HPP
