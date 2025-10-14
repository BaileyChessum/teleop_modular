// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
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
#include "input_ptr.hpp"
#include "input_collection.hpp"
#include "event/event_collection.hpp"

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

  EventCollection & events;
};

/**
 * \class Base class for a control mode used in teleoperation.
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

    /**
     * Whether this control mode should be activated on startup.
     * Actual parameter name is "active".
     */
    bool start_active;
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
   * \brief Gets the name of the control mode, which the control mode is indexed by.
   */
  [[nodiscard]] const std::string & get_name() const
  {
    return name_;
  }

  /**
   * \brief Gets the underlying lifecycle node used by the control mode
   */
  [[nodiscard]] const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & get_node() const
  {
    return node_;
  }

  /**
   * \brief Gets if the control mode should be locked, checking if the "locked" input button is true. When true, the
   * control mode is expected to tell the control system to do nothing.
   *
   * \returns True when the "locked" input is true
   */
  [[nodiscard]] bool is_locked()
  {
    return locked_.value();
  }

  /**
   * \brief Called after the control_mode is created, allowing the control_mode to set up any structures that need to
   * exist for the entire lifecycle of the control_mode.
   */
  virtual return_type on_init()
  {
    return return_type::OK;
  }

  /**
   * \brief Called after on_init, allows the node to get any configuration from node parameters or otherwise.
   *
   * If the configuration cannot be successfully achieved it will transition to ErrorProcessing calling on_error().
   *
   * \param previous_state The previous lifecycle state being transitioned from.
   */
  CallbackReturn on_configure(const State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  /**
   * \brief Exposes inputs to the control mode so that it can capture shared pointers to any inputs it needs. Called
   * after configure.
   *
   * \param inputs References to the collection of buttons and axes to be used by the control_mode.
   */
  void configure_inputs(Inputs inputs);

  /**
   * \brief Called after on_configure, allows the implementer to capture shared pointers to any inputs needed by the
   * control mode.
   *
   * \param inputs References to the collection of buttons and axes to be used by the control_mode.
   */
  virtual void on_configure_inputs(Inputs inputs) = 0;

  /**
   * \brief Called when there are new inputs available. The implementer should send a message to the control system
   * based on input values.
   *
   * \param now   The theoretical current time, based on the time the inputs were received.
   * \param period    The difference in time since the last update.
   */
  virtual return_type on_update(const rclcpp::Time & now, const rclcpp::Duration & period) = 0;

  /**
   * \brief This method is expected to do any final preparations to start executing. This may include acquiring
   * resources that are only held while the node is actually active, such as access to hardware. Ideally, no preparation
   * that requires significant time (such as lengthy hardware initialisation) should be performed in this callback.
   *
   * If this cannot be successfully achieved it will transition to ErrorProcessing calling on_error().
   *
   * \param previous_state The previous lifecycle state being transitioned from.
   */
  CallbackReturn on_activate(const State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  /**
   * \brief This method is expected to clear all state and return the node to a functionally equivalent state as when
   * first created. You may also wish to tell dependent control systems to halt as part of this method.
   *
   * If the deactivating cannot be successfully achieved it will transition to ErrorProcessing calling on_error().
   *
   * \param previous_state The previous lifecycle state being transitioned from.
   */
  CallbackReturn on_deactivate(const State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  /**
   * \brief This method is expected to clear all state and return the node to a functionally equivalent state as when
   * first created.
   *
   * If the cleanup cannot be successfully achieved it will transition to ErrorProcessing calling on_error().
   *
   * \param previous_state The previous lifecycle state being transitioned from.
   */
  CallbackReturn on_cleanup(const State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  /**
   * \brief This transition state is where any error can be cleaned up. It is possible to enter this state from any
   * state where user code will be executed. If error handling is successfully completed the node can return to
   * Unconfigured, If a full cleanup is not possible it must fail and the node will transition to Finalized in
   * preparation for destruction.
   *
   * Transitions to ErrorProcessing may be caused by error return codes in callbacks as well as methods within a
   * callback or an uncaught exception.
   *
   * \param previous_state The previous lifecycle state being transitioned from.
   */
  CallbackReturn on_error(const State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  /**
   * \brief This method is expected to do any cleanup necessary before destruction.
   *
   * \param previous_state The previous lifecycle state being transitioned from.
   */
  CallbackReturn on_shutdown(const State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  [[nodiscard]] const State & get_lifecycle_state() const;
  [[nodiscard]] bool is_active() const;

  /// Gets the names of all ros2_control controllers to activate alongside this control mode.
  [[nodiscard]] const std::vector<std::string> & get_controllers() const;
  /// Gets common params for the control mode, including whether the control mode should start active, and controllers
  [[nodiscard]] const CommonParams & get_common_params() const;

protected:
  /// An input button to represent a lock for the control mode. The control mode should tell the control system to halt
  /// when .value() is true
  Button locked_;

private:
  /// The ROS2 node created by teleop_modular, which we get params from (for base and child classes)
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  /// Name of the control mode, which the control mode is indexed by
  std::string name_;

  /// The params common to all control_modes, managed externally
  CommonParams common_params_;
};

}  // namespace control_mode

#endif  // TELEOP_MODULAR_CONTROL_MODE_HPP
