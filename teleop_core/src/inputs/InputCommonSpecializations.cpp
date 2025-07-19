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
// Created by nova on 7/1/25.
//

#include "teleop_core/inputs/InputCommon.hpp"
#include "teleop_core/inputs/Button.hpp"
#include "teleop_core/inputs/Axis.hpp"

namespace
{
/// This value determines the difference in values in an axis that causes changed() to return true
constexpr float EPSILON = 1e-1f;
}

namespace teleop
{

template<>
bool InputCommon<bool>::value()
{
  return accumulate_value();
}

template<>
uint8_t InputCommon<uint8_t>::value()
{
  return accumulate_value();
}

template<>
double InputCommon<double>::value()
{
  return accumulate_value();
}

template<>
float InputCommon<float>::value()
{
  return accumulate_value();
}

template<>
void InputCommon<uint8_t>::debounce(const rclcpp::Time & now)
{
  previous_debounce_value_ = current_debounce_value_;
  current_debounce_value_ = value();
}

template<>
void InputCommon<float>::debounce(const rclcpp::Time & now)
{
  previous_debounce_value_ = current_debounce_value_;
  // Only change if the change is big enough
  if (const float new_value = value(); std::abs(previous_debounce_value_ - new_value) > EPSILON) {
    current_debounce_value_ = new_value;
  }
}

}  // namespace teleop
