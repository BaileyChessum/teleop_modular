//
// Created by Bailey Chessum on 4/6/25.
//

#ifndef TELEOP_MODULAR_CONTROL_MODE_INPUT_INTERFACE_HPP
#define TELEOP_MODULAR_CONTROL_MODE_INPUT_INTERFACE_HPP

#include <rclcpp/time.hpp>
#include <utility>
#include <string>
#include "visibility_control.h"

namespace control_mode
{

/**
 * @class An interface for objects providing input values to control modes.
 */
template<typename T>
class CONTROL_MODE_PUBLIC InputInterface
{
public:
  using SharedPtr = std::shared_ptr<InputInterface<T>>;
  using WeakPtr = std::weak_ptr<InputInterface<T>>;

  virtual T value() = 0;

  // Accessors
  [[nodiscard]] const std::string & get_name() const
  {
    return name_;
  }

  // Type conversion
  constexpr inline operator T()
  {
    return value();
  }

protected:
  explicit InputInterface(std::string name)
  : name_(std::move(name))
  {
  }
  ~InputInterface() = default;

private:
  const std::string name_;
};

/// Provides access to boolean inputs. Store as a Button::SharedPtr.
using CONTROL_MODE_PUBLIC_TYPE Button = InputInterface<uint8_t>;
/// Provides access to floating point number inputs. Store as an Axis::SharedPtr.
using CONTROL_MODE_PUBLIC_TYPE Axis = InputInterface<float>;

}  // namespace control_mode

#endif  // TELEOP_MODULAR_CONTROL_MODE_INPUT_INTERFACE_HPP
