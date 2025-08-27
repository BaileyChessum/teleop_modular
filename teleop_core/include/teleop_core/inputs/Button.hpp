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

#ifndef TELEOP_MODULAR_BUTTON_HPP
#define TELEOP_MODULAR_BUTTON_HPP

#include <utility>
#include "InputCommon.hpp"
#include "control_mode/input_ptr.hpp"

namespace teleop
{

class Button final : public InputCommon<uint8_t>
{
public:
  using SharedPtr = std::shared_ptr<Button>;
  using WeakPtr = std::weak_ptr<Button>;

  using ControlModeType = control_mode::Button;

  explicit Button(std::string name)
  : InputCommon<uint8_t>(std::move(name))
  {
  }
};

}  // namespace teleop

#endif  // TELEOP_MODULAR_BUTTON_HPP
