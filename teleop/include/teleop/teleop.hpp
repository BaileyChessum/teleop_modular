/**
 * @file teleop.hpp
 * @brief Header file for the TeleopArmJoy class, which handles joystick input for teleoperation of the robotic arm.
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

namespace teleop
{
  /**
   * @class TeleopArmJoy
   * @brief TODO
   */
  class TeleopArmJoy : public rclcpp::Node, public CommandDelegate {
  public:
    /**
     * @brief Constructor for TeleopArmJoy.
     * @param options Node options for the ROS2 node.
     */
    explicit TeleopArmJoy(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~TeleopArmJoy() override;

    void initialize(const std::weak_ptr<rclcpp::Executor>& executor, const std::shared_ptr<TeleopArmJoy>& self);

    void log_all_inputs();

    /**
     * Infinite loop that repeatedly services updates from input sources. The heart of the program.
     */
    [[noreturn]] void service_input_updates();

    [[nodiscard]] std::shared_ptr<const Node> get_node() const override;
    [[nodiscard]] const InputManager& get_inputs() const override;
    [[nodiscard]] const std::shared_ptr<ControlModeManager> get_control_modes() const override;

  private:
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    std::shared_ptr<ControlModeManager> control_mode_manager_ = nullptr;
    InputManager inputs_;
    std::shared_ptr<InputSourceManager> input_source_manager_ = nullptr;
    std::shared_ptr<CommandManager> commands_ = nullptr;

    bool locked_ = true;
  };

} // namespace teleop

#endif // TELEOP_HPP
