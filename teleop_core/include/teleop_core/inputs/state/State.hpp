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

  // Dereference to get a pointer
  T* operator*() { return &value; }
  const T* operator*() const { return &value; }
};

}  // namespace teleop_modular

#endif  // STATE_HPP
