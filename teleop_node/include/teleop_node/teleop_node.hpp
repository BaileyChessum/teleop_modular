// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#ifndef TELEOP_MODULAR_HPP
#define TELEOP_MODULAR_HPP

#include <rclcpp/rclcpp.hpp>

#include "teleop_core/control_modes/control_mode_manager.hpp"
#include "teleop_core/input_sources/InputSourceManager.hpp"
#include "teleop_core/commands/CommandManager.hpp"
#include "teleop_core/inputs/state/StateManager.hpp"
#include "teleop_core/events/EventManager.hpp"
#include "teleop_core/inputs/input_pipeline_builder.hpp"
#include "input_change_listener.hpp"

namespace teleop
{

/**
 * @class TeleopNode
 * @brief Class that uses a ROS2 node to construct and operate the teleop_modular framework.
 */
class TeleopNode : public CommandDelegate, public std::enable_shared_from_this<TeleopNode>
{
public:
  /**
   * @brief Constructor for TeleopNode.
   * @param node The ROS2 node to use for parameters, topics, and services
   */
  explicit TeleopNode(const std::shared_ptr<rclcpp::Node> & node);

  ~TeleopNode() override;

  void initialize(const std::weak_ptr<rclcpp::Executor> & executor);
  void log_existing_inputs();

  /**
   * Infinite loop that repeatedly services updates from input sources. The heart of the program.
   */
  void service_input_updates();

  [[nodiscard]] std::shared_ptr<rclcpp::Node> get_node() const override;
  [[nodiscard]] InputManager & get_inputs() override;
  [[nodiscard]] state::StateManager & get_states() override;
  [[nodiscard]] const std::shared_ptr<internal::ControlModeManager> get_control_modes() const
  override;

  /**
   * Ends any running threads
   */
  void stop();

private:
  struct Params
  {
    bool log_inputs;
    double update_rate;
  };

  std::shared_ptr<rclcpp::Node> node_;

  Params params_;

  InputManager inputs_;
  state::StateManager states_;
  InputPipelineBuilder pipeline_ = InputPipelineBuilder(inputs_);

  internal::EventManager events_;
  std::shared_ptr<internal::CommandManager> commands_ = nullptr;

  std::shared_ptr<internal::ControlModeManager> control_mode_manager_ = nullptr;
  std::shared_ptr<internal::InputSourceManager> input_source_manager_ = nullptr;

  /// Prints changes to inputs when log_inputs:=true
  std::optional<InputChangeListener> input_change_listener_ = std::nullopt;

  std::atomic<bool> program_running_ = true;
};

}  // namespace teleop

#endif  // TELEOP_MODULAR_HPP
