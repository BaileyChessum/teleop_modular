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
#include <memory>
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
  const T& operator*() const { return *ptr_; }
  const T* operator->() const { return ptr_; }
  /// Assume valid pointers will always have a valid name set
  explicit operator bool() const { return name_ && !name_->empty() && ptr_ != &default_value(); }

  // Accessors
  [[nodiscard]] constexpr const std::string & get_name() const noexcept
  {
    if (!*this)
      return *default_name();
    return *name_;
  }

  // Type conversion
  constexpr inline explicit operator T()
  {
    return *ptr_;
  }

  /// Resets the InputPtr to a known safe 'null' value
  void reset() {
    *this = nullptr;
  }

  /**
   * Like a shared_ptr's get method, returns the underlying T* pointer.
   */
  constexpr inline const T* get() const noexcept {
    return ptr_;
  }

  /// This is left over from before the input class used to represent a pointer
  using SharedPtr [[deprecated("Shared pointers are no longer used for input memory management. The input class now acts as the pointer.")]] = InputPtr<T>*;

  InputPtr(InputPtr<T>& other) : ptr_(other.ptr_), name_(other.name_) {}
  
  /// Creates a null input
  InputPtr() {
    ptr_ = &default_value();
    name_ = default_name();
  }
  InputPtr(std::nullptr_t) {
    ptr_ = &default_value();
    name_ = default_name();
  }

  InputPtr& operator=(std::nullptr_t) noexcept {
    ptr_ = &default_value();
    name_ = default_name();
    return *this;
  }

  explicit InputPtr(std::shared_ptr<std::string> name, T* ptr)
  : name_(std::move(name)), ptr_(ptr)
  {
  }

private:
  T* ptr_;
  std::shared_ptr<std::string> name_;

  // Shared mutable default value
  static T& default_value() {
    // mutable T shared between all input pointers
    static T value{};  
    return value;
  }
  static std::shared_ptr<std::string>& default_name() {
    static std::shared_ptr<std::string> name = std::make_shared<std::string>("[null]");
    return name;
  }
};

/// Provides access to boolean inputs. Store as a Button.
using CONTROL_MODE_PUBLIC_TYPE Button = InputPtr<uint8_t>;
/// Provides access to floating point number inputs. Store as an Axis.
using CONTROL_MODE_PUBLIC_TYPE Axis = InputPtr<float>;

}  // namespace control_mode

#endif  // TELEOP_MODULAR_CONTROL_MODE_INPUT_INTERFACE_HPP
