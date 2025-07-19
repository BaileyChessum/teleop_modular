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
// Created by Bailey Chessum on 7/6/25.
//

#ifndef TELEOP_MODULAR_INPUT_SOURCE_INPUT_DECLARATION_LIST_HPP
#define TELEOP_MODULAR_INPUT_SOURCE_INPUT_DECLARATION_LIST_HPP

#include "visibility_control.h"
#include "input_source/utilities/vector_ref.hpp"
#include <vector>
#include <string>

namespace input_source
{

/**
 * A class to be given to InputSource implementation to declare their buttons and axes, in a way that allows button
 * values to be placed next to each other in memory
 */
template<typename T>
class INPUT_SOURCE_PUBLIC InputDeclarationList
{
public:
  using Ref = VectorRef<T>;

  InputDeclarationList(std::vector<std::string> & names, std::vector<T> & values)
  : names_(names), values_(values)
  {
  }

  /**
   * @brief Declares a button/axis to exist with the given name, at an index equal to the number of declarations before
   * calling this method.
   * @param[in] name    The name of the button/axis being declared.
   * @returns   A reference to the value of the declared button/axis. Assign the value stored in this VectorRef with
   * `operator=` during on_update() to change the value of the declared input. Alternatively, you can set the value
   * directly in your InputSource by assigning `this->buttons_[i]` or `this->axes_[i]` to avoid a layer of indirection.
   */
  inline Ref add(const std::string & name) noexcept
  {
    auto idx = values_.size();
    names_.emplace_back(name);
    values_.emplace_back(0);
    return Ref(values_, idx);
  }

  /**
   * @brief Declares a button/axis to exist with the given name, at an index equal to the number of declarations before
   * calling this method.
   * @param[in] name    The name of the button/axis being declared.
   * @param[in] initial_value The initial value to give to this input. This is usually very unnecessary to specify.
   * @returns   A reference to the value of the declared button/axis. Assign the value stored in this VectorRef with
   * `operator=` during on_update() to change the value of the declared input. Alternatively, you can set the value
   * directly in your InputSource by assigning `this->buttons_[i]` or `this->axes_[i]` to avoid a layer of indirection.
   */
  inline Ref add(const std::string & name, T initial_value) noexcept
  {
    auto idx = values_.size();
    names_.emplace_back(name);
    values_.emplace_back(initial_value);
    return Ref(values_, idx);
  }

  /**
   * This attempts to reserve enough memory for the name and value vectors. Please call this before adding values.
   */
  inline void reserve(size_t n) noexcept
  {
    names_.reserve(n);
    values_.reserve(n);
  }

private:
  std::vector<std::string> & names_;
  std::vector<T> & values_;
};

}  // namespace input_source

#endif  // TELEOP_MODULAR_INPUT_SOURCE_INPUT_DECLARATION_LIST_HPP
