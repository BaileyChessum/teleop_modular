/**
 * @file teleop.hpp
 * @brief Header file for the Teleop class, which handles joystick input for teleoperation of the robotic arm.
 * Last Edited by Abby
 */
#ifndef TELEOP_HPP
#define TELEOP_HPP

#include <rclcpp/rclcpp.hpp>

// generate_parameter_library_cpp include/teleop/teleop_parameters.hpp src/parameters.yaml
#include "teleop_parameters.hpp"
#include "control_modes/ControlModeManager.hpp"
#include "input_sources/InputSourceManager.hpp"
#include "commands/CommandManager.hpp"
#include "inputs/state/StateManager.hpp"

namespace teleop
{
  /**
   * @class Teleop
   * @brief TODO
   */
  class Teleop : public CommandDelegate, public std::enable_shared_from_this<Teleop> {
  public:
    /**
     * @brief Constructor for Teleop.
     * @param node The ROS2 node to use for parameters, topics, and services
     * @param options Node options for the ROS2 node.
     */
    explicit Teleop(const std::shared_ptr<rclcpp::Node>& node);

    ~Teleop() override;

    void initialize(const std::weak_ptr<rclcpp::Executor>& executor);
    void log_all_inputs();

    /**
     * Infinite loop that repeatedly services updates from input sources. The heart of the program.
     */
    void service_input_updates();

    [[nodiscard]] std::shared_ptr<rclcpp::Node> get_node() const override;
    [[nodiscard]] const InputManager& get_inputs() const override;
    [[nodiscard]] StateManager& get_states() override;
    [[nodiscard]] const std::shared_ptr<ControlModeManager> get_control_modes() const override;

    /**
     * Ends any running threads
     */
    void stop();

  private:
    std::shared_ptr<rclcpp::Node> node_;

    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    InputManager inputs_;
    StateManager states_;

    std::shared_ptr<ControlModeManager> control_mode_manager_ = nullptr;
    std::shared_ptr<InputSourceManager> input_source_manager_ = nullptr;
    std::shared_ptr<CommandManager> commands_ = nullptr;

    std::atomic<bool> program_running_ = true;
  };

} // namespace teleop

#endif // TELEOP_HPP
