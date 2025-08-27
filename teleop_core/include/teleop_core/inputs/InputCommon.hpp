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

#ifndef TELEOP_MODULAR_INPUTCOMMON_HPP
#define TELEOP_MODULAR_INPUTCOMMON_HPP

#include <rclcpp/time.hpp>
#include <utility>
#include "control_mode/input_interface.hpp"

namespace teleop
{

template<typename T>
class InputCommon : public control_mode::InputInterface<T>
{
public:
  virtual ~InputCommon() = default;

  T value() override
  {
    if (!definition)
      return 0;
    return *definition;
  }

  void debounce(const rclcpp::Time & now);

  /**
   * @returns true if the value changed since last debounce
   */
  [[nodiscard]] bool changed() const;

  /**
   * Adds the given definition to the input
   */
  void add_definition(T* new_definition)
  {
    definition = new_definition;
  }

  /**
   * Removes the first instance of the given definition
   */
  void remove_definition()
  {
    definition = nullptr;
  }

  /// Pointers obtained through InputDefinition<T>s, that provide the value for the input
  T* definition = nullptr;

protected:
  // Constructor is protected, as to ensure plugin classes don't accidentally use this type directly
  explicit InputCommon(std::string name)
  : control_mode::InputInterface<T>(std::move(name))
  {
  }

private:
  struct Params
  {
    T default_value = 0;
  };

  Params params_;

  T previous_debounce_value_ = 0;
  T current_debounce_value_ = 0;
};

template<typename T>
bool InputCommon<T>::changed() const
{
  return previous_debounce_value_ != current_debounce_value_;
}

}  // namespace teleop

#endif  // TELEOP_MODULAR_INPUTCOMMON_HPP
