//
// Created by nova on 7/1/25.
//

#ifndef TELEOP_MODULAR_AXIS_HPP
#define TELEOP_MODULAR_AXIS_HPP

#include <utility>
#include "Input.hpp"

namespace teleop_modular
{

class Axis : public InputCommon<float>
{
public:
  using SharedPtr = std::shared_ptr<Axis>;
  using WeakPtr = std::weak_ptr<Axis>;

  explicit Axis(std::string name) : InputCommon<float>(std::move(name))
  {
  }
};

}  // namespace teleop_modular

#endif  // TELEOP_MODULAR_AXIS_HPP
