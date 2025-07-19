// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Created by nova on 7/4/25.
//

#ifndef STATE_HPP
#define STATE_HPP

#include <memory>
#include <string>
#include "InputDeclaration.hpp"

namespace teleop::state
{

/// Same as a InputDeclaration, but it actually holds the value being referenced too.
template<typename T>
struct State : InputDeclaration<T>
{
  using SharedPtr = std::shared_ptr<State>;

  /// Value offered up as the declaration.
  T value;
  // TODO: Timeout

  State(const std::string & name, T initial_value)
  : InputDeclaration<T>(name, value), value(initial_value)
  {
  }
};

}  // namespace teleop_modular

#endif  // STATE_HPP
