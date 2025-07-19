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
// Created by nova on 6/28/25.
//

#ifndef TELEOP_MODULAR_COMMANDMANAGER_HPP
#define TELEOP_MODULAR_COMMANDMANAGER_HPP

#include <utility>

#include "Command.hpp"
#include "teleop_core/events/EventCollection.hpp"
#include <map>
#include <string>
#include <memory>
#include <vector>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>

namespace teleop::internal
{

class CommandManager final
{
public:
  explicit CommandManager(
    const std::shared_ptr<rclcpp::Node> & node,
    CommandDelegate::WeakPtr context)
  : node_(node), context_(std::move(context))
  {
  }

  /**
   * Tries to create and add a command of a given name, using parameters in node_
   * @param name The name of the command to add
   * @param inputs
   */
  void create_command(const std::string & name, EventCollection & events);

  /**
   * Populates the sources_ from the params in node_.
   */
  void configure(EventCollection & events);

  /**
   * Gets the command plugin class type name for a given command name, to be given to pluginlib to load.
   * Declares the necessary parameter to get the type name as a side effect.
   * @param[in]  name The name of the command to get the plugin type name for.
   * @param[out] source_type The output command plugin type name to be given to pluginlib.
   * @param[out] invocation_event_names The names of the events the command should be invoked by.
   * @return True if the type name was found. False otherwise.
   */
  bool get_type_for_command(
    const std::string & name, std::string & source_type,
    std::vector<std::string> & invocation_event_names) const;

  std::shared_ptr<Command> operator[](const std::string & index);

  void add(const std::string & key, const std::shared_ptr<Command> & value);

  using iterator = typename std::map<std::string, std::shared_ptr<Command>>::iterator;
  using const_iterator = typename std::map<std::string, std::shared_ptr<Command>>::const_iterator;

  iterator begin()
  {
    return items_.begin();
  }

  iterator end()
  {
    return items_.end();
  }

  [[nodiscard]] const_iterator begin() const
  {
    return items_.begin();
  }

  [[nodiscard]] const_iterator end() const
  {
    return items_.end();
  }

private:
  /// The owning teleop_modular ROS2 node.
  std::shared_ptr<rclcpp::Node> node_;

  /// Loads the control modes, and needs to stay alive during the whole lifecycle of the control modes.
  std::unique_ptr<pluginlib::ClassLoader<Command>> loader_;

  std::map<std::string, std::shared_ptr<Command>> items_{};

  CommandDelegate::WeakPtr context_;
};

}  // namespace teleop::internal

#endif  // TELEOP_MODULAR_COMMANDMANAGER_HPP
