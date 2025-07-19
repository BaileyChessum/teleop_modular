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

#ifndef TELEOP_MODULAR_AXIS_HPP
#define TELEOP_MODULAR_AXIS_HPP

#include <utility>
#include "InputCommon.hpp"

namespace teleop
{

class Axis : public InputCommon<float>
{
public:
  using SharedPtr = std::shared_ptr<Axis>;
  using WeakPtr = std::weak_ptr<Axis>;

  using ControlModeType = control_mode::Axis;

  explicit Axis(std::string name)
  : InputCommon<float>(std::move(name))
  {
  }
};

}  // namespace teleop

#endif  // TELEOP_MODULAR_AXIS_HPP
