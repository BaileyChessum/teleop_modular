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

#ifndef TELEOP_MODULAR_INPUTDECLARATION_HPP
#define TELEOP_MODULAR_INPUTDECLARATION_HPP

namespace teleop::state
{

/**
 * The struct used by input sources to export their inputs. Fundamentally, an std::reference_wrapper<T> of the actual
 * data we care about, tied with a name, and any other additional information we might include in the future, like a
 * description.
 */
template<typename T>
struct InputDefinition
{
  std::string name;
  T* reference;

  InputDefinition(std::string name, T* reference)
  : name(std::move(name)), reference(reference)
  {
  }
};

}  // namespace teleop::state

#endif  // TELEOP_MODULAR_INPUTDECLARATION_HPP
