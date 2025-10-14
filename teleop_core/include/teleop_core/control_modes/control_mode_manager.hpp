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
// Created by nova on 6/4/25.
//

#ifndef TELEOP_MODULAR_CONTROLMODEMANAGER_HPP
#define TELEOP_MODULAR_CONTROLMODEMANAGER_HPP

#include <map>
#include <string>
#include <rclcpp/node.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/executor.hpp>
#include "control_mode/control_mode.hpp"
#include "teleop_core/inputs/InputManager.hpp"
#include "teleop_core/inputs/input_pipeline_builder.hpp"
#include "control_mode/event/event_collection.hpp"
#include "teleop_core/control_modes/controller_manager_manager.hpp"

namespace teleop::internal
{

/**
 * Class responsible for managing the registered control modes, the current control mode, and switching between them.
 */
class ControlModeManager final : public InputPipelineBuilder::Element
{
public:
  explicit ControlModeManager(
    const std::shared_ptr<rclcpp::Node> & node,
    const std::weak_ptr<rclcpp::Executor> & executor,
    control_mode::EventCollection & events)
  : node_(node), executor_(executor), events_(events)
  {
  }

  /**
   * Populates the control_modes_ from the params in node_.
   */
  void configure();

  /**
   * @brief Attempts to activate a control mode.
   * @param activate The names of the control modes to activate.
   * @param deactivate The names of the control modes to deactivate.
   * @return True if successfully switched to the control modes.
   */
  bool switch_control_mode(const std::vector<std::string> & activate, const std::vector<std::string> & deactivate);

  /**
   * @brief Attempts to activate a control mode.
   * @param name The name of the control mode to load.
   * @return True if successfully switched to the control mode.
   */
  bool set_control_mode(const std::string & name);

  /**
   * Update the active node
   */
  auto update(const rclcpp::Time & now, const rclcpp::Duration & period) const -> void;

  // Collection<ControlMode> implementation
  std::shared_ptr<control_mode::ControlMode> operator[](const std::string & index);

  using iterator = typename std::map<std::string,
      std::shared_ptr<control_mode::ControlMode>>::iterator;
  using const_iterator = typename std::map<std::string,
      std::shared_ptr<control_mode::ControlMode>>::const_iterator;

  iterator begin()
  {
    return control_modes_.begin();
  }
  [[nodiscard]] const_iterator begin() const
  {
    return control_modes_.begin();
  }
  iterator end()
  {
    return control_modes_.end();
  }
  [[nodiscard]] const_iterator end() const
  {
    return control_modes_.end();
  }

  void add(const std::string & key, const std::shared_ptr<control_mode::ControlMode> & value);

  void activate_initial_control_modes();

  /**
     * Add inputs to the builder.
     * \param[in] previous The result of the previous InputPipelineBuilder::Element, to use as a basis for populating
     * next.
     * \param[in,out] next The result of this Element. Always stores the previous result from this Element.
   */
  void link_inputs(const InputManager::Props& previous, InputManager::Props& next, const InputPipelineBuilder::DeclaredNames& names) override;

  // TODO: Rename to make clear that these are the inputs we want to consume, not provide
  /**
     * Allows an element to declare what inputs it CONSUMES, not provides. This is useful for any dynamic remapping of
     * any previous elements in the pipeline.
     * \param[in, out] names the set accumulating all declared input names. Add names to declare to this set.
   */
  void declare_input_names(InputPipelineBuilder::DeclaredNames& names) override;

  /**
     * Callback ran when hardened inputs are available.
   */
  void on_inputs_available(InputManager::Hardened& inputs) override;

private:
  /**
   * Resets everything for the controller manager
   */
  void reset();

  /**
   * Switches ros2_control controllers in the controller_manager for the given change in control modes.
   * @param controllers_to_deactivate the control mode being deactivated.
   * @param controllers_to_activate the control mode being activated.
   * @return True if the request was made successfully. False otherwise.
   */
  [[nodiscard]] bool switch_controllers(
    const std::vector<std::string> & controllers_to_deactivate,
    const std::vector<std::string> & controllers_to_activate) const;

  /**
   * Gets the control mode plugin class type name for a given control mode name, to be given to pluginlib to load.
   * Declares the necessary parameter to get the type name as a side effect.
   * @param[in]  name The name of the control mode to get the plugin type name for.
   * @param[out] control_mode_type The output control mode plugin type name to be given to pluginlib.
   * @return True if the type name was found. False otherwise.
   */
  bool get_type_for_control_mode(const std::string & name, std::string & control_mode_type) const;

  /// The owning teleop_modular ROS2 node.
  std::shared_ptr<rclcpp::Node> node_;
  /// Add spawned nodes to this to get them to spin
  std::weak_ptr<rclcpp::Executor> executor_;
  /// Used to get events from when configuring inputs
  control_mode::EventCollection& events_;

  /// Helps with switching controllers in ros2_control, managing separate threads, error recovery and activation order.
  ControllerManagerManager controllers_ = ControllerManagerManager(node_);

  // Control modes
  /// Loads the control modes, and needs to stay alive during the whole lifecycle of the control modes.
  std::unique_ptr<pluginlib::ClassLoader<control_mode::ControlMode>> control_mode_loader_;

  // TODO: Replace with control_modes_by_id_ and name_to_cm_id_
  /// Currently loaded control modes.
  std::map<std::string, std::shared_ptr<control_mode::ControlMode>> control_modes_{};

  std::vector<std::shared_ptr<control_mode::ControlMode>> control_modes_by_id_{};
  std::map<std::string, size_t> name_to_cm_id_{};
};

}  // namespace teleop::internal

#endif  // TELEOP_MODULAR_CONTROLMODEMANAGER_HPP
