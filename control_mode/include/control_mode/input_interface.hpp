//
// Created by Bailey Chessum on 4/6/25.
//

#ifndef TELEOP_MODULAR_CONTROL_MODE_INPUT_INTERFACE_HPP
#define TELEOP_MODULAR_CONTROL_MODE_INPUT_INTERFACE_HPP

#include <rclcpp/time.hpp>
#include <utility>

namespace control_mode
{

/**
 * @class An interface for objects providing input values to control modes.
 */
template <typename T>
class InputInterface
{
public:
  using SharedPtr = std::shared_ptr<InputInterface>;
  using WeakPtr = std::weak_ptr<InputInterface>;

  virtual T value() = 0;

  // Accessors
  [[nodiscard]] const std::string& get_name() const
  {
    return name_;
  }

  // Type conversion
  constexpr inline operator T()
  {
    return value();
  }

protected:
  explicit InputInterface(std::string name) : name_(std::move(name))
  {
  }

private:
  const std::string name_;
};

/// Provides access to boolean inputs. Store as a Button::SharedPtr.
using Button = InputInterface<uint8_t>;
/// Provides access to floating point number inputs. Store as an Axis::SharedPtr.
using Axis = InputInterface<float>;

}  // namespace control_mode

#endif  // TELEOP_MODULAR_CONTROL_MODE_INPUT_INTERFACE_HPP
