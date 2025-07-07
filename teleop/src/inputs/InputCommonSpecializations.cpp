//
// Created by nova on 7/1/25.
//

#include "teleop/inputs/InputCommon.hpp"
#include "teleop/inputs/Button.hpp"
#include "teleop/inputs/Axis.hpp"

namespace
{
/// This value determines the difference in values in an axis that causes changed() to return true
constexpr float EPSILON = 1e-1f;
}

namespace teleop
{

template <>
bool InputCommon<bool>::value()
{
  return accumulate_value();
}

template <>
uint8_t InputCommon<uint8_t>::value()
{
  return accumulate_value();
}

template <>
double InputCommon<double>::value()
{
  return accumulate_value();
}

template <>
float InputCommon<float>::value()
{
  return accumulate_value();
}

template <>
void InputCommon<uint8_t>::debounce(const rclcpp::Time& now)
{
  previous_debounce_value_ = current_debounce_value_;
  current_debounce_value_ = value();
};

template <>
void InputCommon<float>::debounce(const rclcpp::Time& now)
{
  previous_debounce_value_ = current_debounce_value_;
  // Only change if the change is big enough
  if (const float new_value = value(); std::abs(previous_debounce_value_ - new_value) > EPSILON)
    current_debounce_value_ = new_value;
};

}  // namespace teleop
