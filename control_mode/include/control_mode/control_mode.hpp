//
// Created by Bailey Chessum on 6/4/25.
//

#ifndef TELEOP_MODULAR_CONTROL_MODE_HPP
#define TELEOP_MODULAR_CONTROL_MODE_HPP

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/executor.hpp>
#include "visibility_control.h"
#include "input_interface.hpp"
#include "input_collection.hpp"

namespace control_mode
{

/**
 * To replace returning bools to indicate whether an operation was successful or failed due to some error. Used to
 * be more explicit.
 */
enum class CONTROL_MODE_PUBLIC_TYPE return_type : bool
{
  OK = false,
  ERROR = true,
};

// These usings are added with hopes to reduce the length of type names for implementers of control modes
/// The return type of LifecycleNodeInterface callback
using CONTROL_MODE_PUBLIC_TYPE CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using CONTROL_MODE_PUBLIC_TYPE State = rclcpp_lifecycle::State;

/**
 * @brief Provides references to all collections of input objects that might be needed in a control_mode.
 *
 * Including this extra struct affords abstraction for the providers of buttons, axes, and the potential expansion of
 * available inputs (events?)
 */
struct CONTROL_MODE_PUBLIC_TYPE Inputs
{
  ButtonCollection & buttons;
  AxisCollection & axes;
};

/**
 * Base class for a control mode used in teleoperation.
 */
class CONTROL_MODE_PUBLIC ControlMode : public rclcpp_lifecycle::node_interfaces::
  LifecycleNodeInterface
{
public:
  /**
   * These are parameters shared across all controllers. These can be populated by setting the control_mode_name.
   */
  struct CommonParams
  {
    /**
     * The set of ros2_control controllers to activate with this control mode. Populated by the
     * control_modes.control_mode_name.controllers parameter in the main teleop_modular node.
     */
    std::vector<std::string> controllers;
  };

  ~ControlMode() override;
  ControlMode() = default;

  /**
   * This function effectively replaces the initializer, and should only be called once immediately after spawning the
   * control mode, and before anything else.
   */
  return_type init(
    const std::string & name, const std::string & node_namespace,
    const rclcpp::NodeOptions & node_options,
    const std::shared_ptr<rclcpp::Executor> & executor, const CommonParams & common_params);

  // Accessors
  /**
   * Gets the name of the control mode, which the control mode is indexed by.
   */
  [[nodiscard]] const std::string & get_name() const
  {
    return name_;
  }
  /**
   * Gets the underlying lifecycle node used by the control mode
   */
  [[nodiscard]] const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & get_node() const
  {
    return node_;
  }

  virtual return_type on_init()
  {
    return return_type::OK;
  }
  virtual void capture_inputs(Inputs inputs) = 0;
  virtual return_type update(const rclcpp::Time & now, const rclcpp::Duration & period) = 0;

  CallbackReturn on_activate(const State &) override
  {
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const State &) override
  {
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_cleanup(const State &) override
  {
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_error(const State &) override
  {
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_configure(const State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  [[nodiscard]] const State & get_lifecycle_state() const;
  [[nodiscard]] bool is_active() const;

  /// Gets the names of all ros2_control controllers to activate alongside this control mode.
  [[nodiscard]] const std::vector<std::string> & get_controllers() const;

protected:
  /// The ROS2 node created by teleop_modular, which we get params from (for base and child classes)
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

private:
  /// Name of the control mode, which the control mode is indexed by
  std::string name_;

  /// The params common to all control_modes, managed externally
  CommonParams common_params_;
};

}  // namespace control_mode

#endif  // TELEOP_MODULAR_CONTROL_MODE_HPP
