// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Created by nova on 6/28/25.
//

#ifndef TELEOP_MODULAR_COMMANDDELEGATE_HPP
#define TELEOP_MODULAR_COMMANDDELEGATE_HPP
#include <memory>
#include <rclcpp/node.hpp>

#include "teleop_core/inputs/InputManager.hpp"
#include "teleop_core/inputs/state/StateManager.hpp"

namespace teleop
{

namespace internal
{
class ControlModeManager;  // Forward declaration
}

/**
 * Delegate interface allowing Actions to access the internals of the teleop_modular node in a controlled manner.
 */
class CommandDelegate
{
public:
  using WeakPtr = std::weak_ptr<CommandDelegate>;
  using SharedPtr = std::shared_ptr<CommandDelegate>;

  virtual ~CommandDelegate() = default;

  [[nodiscard]] virtual std::shared_ptr<rclcpp::Node> get_node() const = 0;
  [[nodiscard]] virtual const InputManager & get_inputs() const = 0;
  [[nodiscard]] virtual state::StateManager & get_states() = 0;
  [[nodiscard]] virtual const std::shared_ptr<internal::ControlModeManager> get_control_modes()
  const = 0;
};

}  // namespace teleop

#endif  // TELEOP_MODULAR_COMMANDDELEGATE_HPP
