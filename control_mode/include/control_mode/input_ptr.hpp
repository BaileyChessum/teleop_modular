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
class CONTROL_MODE_PUBLIC InputPtr
{
public:
  constexpr const T& value() const noexcept {
    return *ptr_;
  }

  // Pointer-like operators
  T& operator*() const { return *ptr_; }
  T* operator->() const { return ptr_; }
  /// Assume valid pointers will always have a valid name set
  explicit operator bool() const { return name_ && !name_->empty(); }

  // Accessors
  [[nodiscard]] constexpr const std::string & get_name() const noexcept
  {
    if (!*this)
      return std::string("[null]");
    return *name_;
  }

  // Type conversion
  constexpr inline explicit operator T()
  {
    return value();
  }

  /// This is left over from before the input class used to represent a pointer
  struct [[deprecated("Shared pointers are no longer used for input memory management. The input class now acts as the pointer.")]]
  SharedPtrDeprecationHelper {
    using type = std::shared_ptr<InputPtr<T>>;
  };
  using SharedPtr [[deprecated("Shared pointers are no longer used for input memory management. The input class now acts as the pointer.")]] = typename std::shared_ptr<InputPtr<T>>;

protected:
  explicit InputPtr(std::shared_ptr<std::string> name, T* ptr)
  : name_(std::move(name)), ptr_(ptr)
  {
  }
  ~InputPtr() = default;

private:
  T* ptr_;
  const std::shared_ptr<std::string> name_;
};

/// Provides access to boolean inputs. Store as a Button::SharedPtr.
using CONTROL_MODE_PUBLIC_TYPE Button = InputPtr<uint8_t>;
/// Provides access to floating point number inputs. Store as an Axis::SharedPtr.
using CONTROL_MODE_PUBLIC_TYPE Axis = InputPtr<float>;

}  // namespace control_mode

#endif  // TELEOP_MODULAR_CONTROL_MODE_INPUT_INTERFACE_HPP
