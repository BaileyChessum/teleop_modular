//
// Created by Bailey Chessum on 4/6/25.
//

#ifndef TELEOP_INPUTSOURCE_HPP
#define TELEOP_INPUTSOURCE_HPP

#include <rclcpp/node.hpp>

#include "InputSourceUpdateDelegate.hpp"
#include "teleop/events/Event.hpp"
#include "teleop/inputs/InputCommon.hpp"
#include "teleop/inputs/InputManager.hpp"
#include "teleop/inputs/InputDeclaration.hpp"
#include "teleop/input_sources/InputDeclarationList.hpp"
#include "teleop/utilities/span.hpp"

namespace teleop
{

//class InputSourceManager; // Forward declaration

namespace internal
{
  class InputSourceHandle;  // Forward declaration
}

/**
 * A base class for various sources of inputs and event invokers, such as joysticks, keyboards, the GUI, etc.
 */
class InputSource
{
public:
  struct InputValueSpans
  {
    span<uint8_t> buttons;
    span<float> axes;
  };

  struct InputDeclarationSpans : InputValueSpans
  {
    span<std::string> button_names;
    span<std::string> axis_names;
  };

  virtual ~InputSource() = default;

  void initialize(const std::shared_ptr<rclcpp::Node>& node, const std::string& name,
                  const std::weak_ptr<InputSourceUpdateDelegate>& delegate);
  void update(const rclcpp::Time& now);

  // Accessors
  [[nodiscard]] inline const std::string& get_name() const noexcept
  {
    return name_;
  }

  [[nodiscard]] inline const std::shared_ptr<rclcpp::Node>& get_node() const noexcept
  {
    return node_;
  }

  // TODO: Better access control!

  /// Should only be called by InputSourceManager!
  InputDeclarationSpans export_inputs();

protected:
  /**
   * Called when starting up the input source, allowing the implementation to get configuration from it's node.
   */
  virtual void on_initialize() = 0;

  /**
   * @brief Called after on_configure(), allows the input source to declare memory pointing to button values.
   *
   * @attention Be very careful to make sure you don't make a copy adding to the declarations vector. Everything needs
   * to be a reference to a variable held by your class:
   * @code
   * // This will cause a segfault!!
   * auto button = buttons_.emplace_back(button_name, button_config);
   * declarations.emplace_back(button.name, button.value);
   *
   * // This is good, since we use a reference &
   * auto & button = buttons_.emplace_back(button_name, button_config);
   * declarations.emplace_back(button.name, button.value);
   * @endcode
   * If this causes a segfault, you've likely copied memory by accident.
   *
   * @param[out] declarations Information defining each button exposed by the input source, and a reference to the
   * memory holding the value for that input.
   */
  virtual void export_buttons(InputDeclarationList<uint8_t>& declarations) = 0;

  /**
   * @brief Called after on_configure(), allows the input source to declare memory pointing to axis values.
   *
   * @attention Be very careful to make sure you don't make a copy adding to the declarations vector. Everything needs
   * to be a reference to a variable held by your class:
   * @code
   * // This will cause a segfault!!
   * auto axis = this->axes_.emplace_back(axis_name, axis_config);
   * declarations.emplace_back(axis.name, axis.value);
   *
   * // This is good, since we use a reference &
   * auto& axis = this->axes_.emplace_back(axis_name, axis_config);
   * declarations.emplace_back(axis.name, axis.value);
   * @endcode
   * If this causes a segfault, you've likely copied memory by accident.
   *
   * @param[out] declarations Information defining each button exposed by the input source, and a reference to the
   * memory holding the value for that input.
   */
  virtual void export_axes(InputDeclarationList<float>& declarations) = 0;

  /**
   * Call this whenever you receive a new input and you want the InputManager to invoke a new update.
   */
  bool request_update(const rclcpp::Time& now = rclcpp::Time()) const;

  /**
   * Called when new input should be processed, after requesting an update through request_update().
   * @param[in] now The time of the update, for synchronisation.
   * @param[out] values Modified by the InputSource to set the values for each button and axis.
   */
  virtual void on_update(const rclcpp::Time& now, InputValueSpans values) {};

private:
//  friend class InputSourceManager;
  friend class InputSourceHandle;

  /// The ROS2 node created by teleop, which we get params from (for base and child classes)
  std::shared_ptr<rclcpp::Node> node_ = nullptr;

  std::string name_;
  std::weak_ptr<InputSourceUpdateDelegate> delegate_{};

  std::vector<std::string> button_names_{};
  std::vector<uint8_t> button_values_{};

  std::vector<std::string> axis_names_{};
  std::vector<float> axis_values_{};
};

}  // namespace teleop

#endif  // TELEOP_INPUTSOURCE_HPP
