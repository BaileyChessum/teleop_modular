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
// Created by Bailey Chessum on 4/6/25.
//

#ifndef TELEOP_MODULAR_INPUTSOURCEMANAGER_HPP
#define TELEOP_MODULAR_INPUTSOURCEMANAGER_HPP

#include <queue>
#include <vector>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/node.hpp>

#include "input_source/input_source.hpp"
#include "input_source/update_delegate.hpp"
#include "teleop_core/utilities/SpawnableLog.hpp"
#include "InputSourceHandle.hpp"

namespace teleop::internal
{

class InputSourceManager final : public input_source::UpdateDelegate,
  public std::enable_shared_from_this<input_source::UpdateDelegate>
{
public:
  InputSourceManager() = default;
  explicit InputSourceManager(
    const std::shared_ptr<rclcpp::Node> & node,
    const std::weak_ptr<rclcpp::Executor> & executor, InputManager & inputs)
  : node_(node), executor_(executor), inputs_(std::ref(inputs))
  {
  }

  /**
   * Populates the sources_ from the params in node_.
   */
  void configure(InputManager & inputs);

  /**
   * Gets the control mode plugin class type name for a given input source name, to be given to pluginlib to load.
   * Declares the necessary parameter to get the type name as a side effect.
   * @param[in]  name The name of the input source to get the plugin type name for.
   * @param[out] source_type The output input source plugin type name to be given to pluginlib.
   * @return True if the type name was found. False otherwise.
   */
  bool get_type_for_input_source(const std::string & name, std::string & source_type) const;

  void on_input_source_requested_update(const rclcpp::Time & now) override;

  /**
   * Blocks the current thread until an update is requested by an input source.
   */
  rclcpp::Time wait_for_update();

  void update(const rclcpp::Time & now);

  bool create_input_source(const std::string & input_source_name);

private:
  struct Params
  {
    double min_update_rate = 0.0;
  };


  void setup_input_sources();

  /// The owning teleop_modular ROS2 node.
  std::shared_ptr<rclcpp::Node> node_;
  /// Add spawned nodes to this to get them to spin
  std::weak_ptr<rclcpp::Executor> executor_;
  /// A reference to the input manager used for linking
  std::reference_wrapper<InputManager> inputs_;

  Params params_;

  /// Loads the control modes, and needs to stay alive during the whole lifecycle of the control modes.
  std::unique_ptr<pluginlib::ClassLoader<input_source::InputSource>> source_loader_;

  /// The structure that holds all the tests
  std::vector<internal::InputSourceHandle> sources_{};

  /// Mutex for handling input update requests made by nodes for each input source running on different threads to the
  /// main update thread.
  std::mutex mutex_;
  /// Lock that waits until should_update_ is true.
  std::condition_variable update_condition_;
  std::atomic<bool> should_update_ = false;
  std::queue<std::weak_ptr<input_source::InputSource>> sources_to_update_;
  rclcpp::Time update_time_ = rclcpp::Time();

  /// Used to log the spawned input sources
  std::vector<utils::SpawnableLog> logs_{};
};

}  // namespace teleop::internal

#endif  // TELEOP_MODULAR_INPUTSOURCEMANAGER_HPP
