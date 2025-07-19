//
// Created by Bailey Chessum on 4/6/25.
//

#ifndef TELEOP_MODULAR_INPUT_SOURCE_HPP
#define TELEOP_MODULAR_INPUT_SOURCE_HPP

#include "visibility_control.h"
#include <rclcpp/node.hpp>

#include "input_source/visibility_control.h"
#include "input_source/utilities/span.hpp"
#include "input_source/update_delegate.hpp"
#include "input_source/input_declaration_list.hpp"

namespace teleop::internal
{
class INPUT_SOURCE_PUBLIC_TYPE InputSourceHandle;  // Forward declaration
}

namespace input_source
{

/**
 * To replace returning bools to indicate whether an operation was successful or failed due to some error. Used to
 * be more explicit than just a bool.
 */
enum class INPUT_SOURCE_PUBLIC_TYPE return_type : bool
{
  OK = false,
  ERROR = true
};

struct INPUT_SOURCE_PUBLIC_TYPE InputValueSpans
{
  span<uint8_t> buttons;
  span<float> axes;
};

struct INPUT_SOURCE_PUBLIC_TYPE InputDeclarationSpans : InputValueSpans
{
  span<std::string> button_names;
  span<std::string> axis_names;
};

/**
 * A base class for various sources of inputs and event invokers, such as joysticks, keyboards, the GUI, etc.
 */
class INPUT_SOURCE_PUBLIC InputSource
{
public:
  virtual ~InputSource() = default;

  return_type init(
    const std::shared_ptr<rclcpp::Node> & node, const std::string & name,
    const std::weak_ptr<UpdateDelegate> & delegate);
  return_type update(const rclcpp::Time & now);

  // Accessors
  [[nodiscard]] inline const std::string & get_name() const noexcept
  {
    return name_;
  }

  [[nodiscard]] inline const std::shared_ptr<rclcpp::Node> & get_node() const noexcept
  {
    return node_;
  }

  /// Should only be called by InputSourceManager!
  InputDeclarationSpans export_inputs();

protected:
  /**
   * Called when starting up the input source, allowing the implementation to get configuration from it's node.
   */
  virtual return_type on_init() = 0;

  /**
   * @brief Called after on_configure(), allows the input source to declare the names of all the buttons, creating
   * memory to store the values for each button as a side-effect.
   *
   * When adding names to declarations, a VectorRef object will be returned, pointing to the memory that holds the value
   * for the declared button.
   *
   * @param[out] declarations Information defining each button exposed by the input source.
   */
  virtual void export_buttons(InputDeclarationList<uint8_t> & declarations) = 0;

  /**
   * @brief Called after on_configure(), allows the input source to declare the names of all the axes, creating memory
   * to store the values for each axis as a side-effect.
   *
   * When adding names to declarations, a VectorRef object will be returned, pointing to the memory that holds the value
   * for the declared axis.
   *
   * @param[out] declarations Information defining each axis exposed by the input source.
   */
  virtual void export_axes(InputDeclarationList<float> & declarations) = 0;

  /**
   * Call this whenever you receive a new input and you want the InputManager to invoke a new update.
   */
  return_type request_update(const rclcpp::Time & now = rclcpp::Time()) const;

  /**
   * Called when new input should be processed, after requesting an update through request_update().
   * @param[in] now The time of the update, for synchronisation.
   * @param[out] values Modified by the InputSource to set the values for each button and axis.
   */
  virtual return_type on_update(const rclcpp::Time & now, InputValueSpans values) = 0;

private:
  //  friend class InputSourceManager;
  friend class InputSourceHandle;

  /// The ROS2 node created by teleop_modular, which we get params from (for base and child classes)
  std::shared_ptr<rclcpp::Node> node_ = nullptr;

  std::string name_;
  std::weak_ptr<UpdateDelegate> delegate_{};

  std::vector<std::string> button_names_{};
  std::vector<uint8_t> button_values_{};

  std::vector<std::string> axis_names_{};
  std::vector<float> axis_values_{};
};

}  // namespace input_source

#endif  // TELEOP_MODULAR_INPUT_SOURCE_HPP
