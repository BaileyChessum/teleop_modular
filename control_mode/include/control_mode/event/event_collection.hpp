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
// Created by Bailey Chessum on 01/09/2025.
//

#ifndef TELEOP_MODULAR_CONTROL_MODE_EVENT_COLLECTION_HPP
#define TELEOP_MODULAR_CONTROL_MODE_EVENT_COLLECTION_HPP

#include "control_mode/visibility_control.h"
#include "event.hpp"

namespace control_mode
{

/**
 * ABC for something that provides Event pointers
 */
class CONTROL_MODE_PUBLIC EventCollection
{
public:
  /**
   * @brief Gets or constructs an input with the indexed name.
   *
   * The collection doesn't hold strong ownership of the inputs it returns.
   * As a result, if all external shared pointers to a particular input are dropped, the input object may be destroyed.
   *
   * The collection may create a new input object if none exists for the given name.
   *
   * Names of returned inputs may be remapped, such that (*this)[name]->get_name() may not equal name.
   */
  virtual Event::SharedPtr operator[](const std::string & name) = 0;
};

}  // namespace control_mode

#endif  // TELEOP_MODULAR_CONTROL_MODE_EVENT_COLLECTION_HPP
