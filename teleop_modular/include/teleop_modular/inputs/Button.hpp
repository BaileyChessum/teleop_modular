//
// Created by nova on 7/1/25.
//

#ifndef TELEOP_MODULAR_BUTTON_HPP
#define TELEOP_MODULAR_BUTTON_HPP

#include <utility>
#include "Input.hpp"

namespace teleop_modular
{

class Button final : public InputCommon<uint8_t>
{
public:
  using SharedPtr = std::shared_ptr<Button>;
  using WeakPtr = std::weak_ptr<Button>;

  explicit Button(std::string name) : InputCommon<uint8_t>(std::move(name))
  {
  }
};

}  // namespace teleop_modular

#endif  // TELEOP_MODULAR_BUTTON_HPP
