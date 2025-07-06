//
// Created by nova on 7/1/25.
//

#include "teleop/inputs/InputCommon.hpp"
#include "teleop/inputs/Button.hpp"
#include "teleop/inputs/Axis.hpp"

namespace teleop
{

template <>
bool InputCommon<bool>::value()
{
  return accumulate_value();
}

template <>
double InputCommon<double>::value()
{
  return accumulate_value();
}

}  // namespace teleop
