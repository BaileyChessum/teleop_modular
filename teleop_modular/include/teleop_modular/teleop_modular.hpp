/**
 * @file teleop_modular.hpp
 * @brief Header file for the Teleop_Modular class, which handles joystick input for teleop_modulareration of the robotic arm.
 * Last Edited by Abby
 */
#ifndef TELEOP_MODULAR_HPP
#define TELEOP_MODULAR_HPP

#include <rclcpp/rclcpp.hpp>

// generate_parameter_library_cpp include/teleop_modular/teleop_modular_parameters.hpp src/parameters.yaml
#include "teleop_modular_parameters.hpp"
#include "control_modes/ControlModeManager.hpp"
#include "input_sources/InputSourceManager.hpp"
#include "commands/CommandManager.hpp"
#include "inputs/state/StateManager.hpp"
#include "teleop_modular/events/EventManager.hpp"

namespace teleop
{
/**
 * @class Teleop_Modular
 * @brief TODO
 */
class TeleopModular : public CommandDelegate, public std::enable_shared_from_this<TeleopModular>
{
public:
  /**
   * @brief Constructor for Teleop_Modular.
   * @param node The ROS2 node to use for parameters, topics, and services
   * @param options Node options for the ROS2 node.
   */
  explicit TeleopModular(const std::shared_ptr<rclcpp::Node>& node);

  ~TeleopModular() override;

  void initialize(const std::weak_ptr<rclcpp::Executor>& executor);
  void log_all_inputs();
  void log_existing_inputs();

  /**
   * Infinite loop that repeatedly services updates from input sources. The heart of the program.
   */
  void service_input_updates();

  [[nodiscard]] std::shared_ptr<rclcpp::Node> get_node() const override;
  [[nodiscard]] const InputManager& get_inputs() const override;
  [[nodiscard]] state::StateManager& get_states() override;
  [[nodiscard]] const std::shared_ptr<internal::ControlModeManager> get_control_modes() const override;

  /**
   * Ends any running threads
   */
  void stop();

private:
  std::shared_ptr<rclcpp::Node> node_;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  InputManager inputs_;
  state::StateManager states_;
  internal::EventManager events_;

  std::shared_ptr<internal::ControlModeManager> control_mode_manager_ = nullptr;
  std::shared_ptr<internal::InputSourceManager> input_source_manager_ = nullptr;
  std::shared_ptr<internal::CommandManager> commands_ = nullptr;

  std::atomic<bool> program_running_ = true;
};

}  // namespace teleop

#endif  // TELEOP_MODULAR_HPP
