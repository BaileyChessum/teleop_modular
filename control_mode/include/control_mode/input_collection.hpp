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
// Created by Bailey Chessum on 7/18/25.
//

#ifndef TELEOP_MODULAR_CONTROL_MODE_INPUT_COLLECTION_HPP
#define TELEOP_MODULAR_CONTROL_MODE_INPUT_COLLECTION_HPP

#include "visibility_control.h"
#include "input_interface.hpp"
#include <string>

namespace control_mode
{

/**
 * @class An interface for whatever provides InputInterfaces to control modes.
 *
 * You do not own the lifecycle of the InputCollection. Do not hold a reference to the object. Attempting to use an
 * InputCollection outside of ControlMode::configure_inputs() will cause a segfault.
 */
template<typename InputT>
class CONTROL_MODE_PUBLIC InputCollection
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
  virtual typename InputT::SharedPtr operator[](const std::string & name) = 0;

protected:
  InputCollection() = default;
  ~InputCollection() = default;

  // Prevent copy/move by default
  InputCollection(const InputCollection &) = delete;
  InputCollection(InputCollection &&) = delete;
  InputCollection & operator=(const InputCollection &) = delete;
  InputCollection & operator=(InputCollection &&) = delete;
};

/// A set to acquire shared pointers to boolean inputs
using CONTROL_MODE_PUBLIC_TYPE ButtonCollection = InputCollection<Button>;
/// A set to acquire shared pointers to floating point number inputs
using CONTROL_MODE_PUBLIC_TYPE AxisCollection = InputCollection<Axis>;

}  // namespace control_mode

#endif  // TELEOP_MODULAR_CONTROL_MODE_INPUT_COLLECTION_HPP
