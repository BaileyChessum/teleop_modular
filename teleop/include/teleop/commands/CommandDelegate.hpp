//
// Created by nova on 6/28/25.
//

#ifndef TELEOP_COMMANDDELEGATE_HPP
#define TELEOP_COMMANDDELEGATE_HPP
#include <memory>
#include <rclcpp/node.hpp>

#include "teleop/inputs/InputManager.hpp"

namespace teleop {

class ControlModeManager;  // Forward declaration

/**
 * Delegate interface allowing Actions to access the internals of the teleop node in a controlled manner.
 */
class CommandDelegate {
public:
  using WeakPtr = std::weak_ptr<CommandDelegate>;
  using SharedPtr = std::shared_ptr<CommandDelegate>;

  virtual ~CommandDelegate() = default;

  [[nodiscard]] virtual std::shared_ptr<const rclcpp::Node> get_node() const = 0;
  [[nodiscard]] virtual const InputManager& get_inputs() const = 0;
  [[nodiscard]] virtual const std::shared_ptr<ControlModeManager> get_control_modes() const = 0;
};

} // teleop

#endif //TELEOP_COMMANDDELEGATE_HPP
