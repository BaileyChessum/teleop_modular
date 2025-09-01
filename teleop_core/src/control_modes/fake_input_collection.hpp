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

#ifndef INPUT_CORE_FAKE_INPUT_COLLECTION_HPP
#define INPUT_CORE_FAKE_INPUT_COLLECTION_HPP

#include "control_mode/input_collection.hpp"
#include <set>
#include <string>

namespace teleop::internal
{

/**
 * An input collection that doesn't provide any real values, but collects the names requested by any consumers.
 */
template <typename InputT>
class FakeInputCollection : public control_mode::InputCollection<InputT>
{
public:
  FakeInputCollection() = default;

  /**
   * @brief Gets or constructs an input with the indexed name.
   *
   * The collection may create a new input object if none exists for the given name (or will give you a 'dud' null
   * input, which is still valid to use, but will always be 0).
   *
   * Names of returned inputs may be remapped, such that (*this)[name]->get_name() may not equal name.
   */
  virtual InputT operator[](const std::string & name) override {
    names_.insert(name);
    return InputT();
  }

  /**
   * Gets the set of all input names requested from the collection.
   */
  const std::set<std::string>& get_names() const
  {
    return names_;
  }

private:
  std::set<std::string> names_{};
};

}  // namespace teleop::internal

#endif  // INPUT_CORE_FAKE_INPUT_COLLECTION_HPP
