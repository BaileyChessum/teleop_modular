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
// Created by Bailey Chessum on 8/27/25.
//

#ifndef TELEOP_CORE_INPUTAGGREGATOR_HPP
#define TELEOP_CORE_INPUTAGGREGATOR_HPP

#include <vector>
#include <stdexcept>
#include <variant>

namespace teleop
{

/**
 * Class that adds multiple input values and stores the result in another input
 */
template <typename T>
class InputAggregator
{
public:
  /**
   * Data needed to construct an InputAggregator
   */
  struct Props {
    /// ids of inputs or pointers to inputs to sum
    std::vector<std::variant<size_t, T*>> source_ids;
    /// id of input to store result to
    size_t target_id;
  };

  InputAggregator(std::vector<T>& inputs, const Props& props) {
    if (inputs.empty())
      throw std::invalid_argument("Inputs cannot have a length of zero. It is missing the null value at id 0.");
    if (props.source_ids.empty())
      throw std::invalid_argument("props.source_ids needs to have at least one element.");

    // Assign members from props
    sources_.reserve(props.source_ids.size());
    for (auto source_id : props.source_ids) {
      if (std::holds_alternative<size_t>(source_id)) {
        size_t idx = std::get<size_t>(source_id);
        if (idx >= inputs.size())
          throw std::out_of_range("source_id out of bounds");
        sources_.push_back(&inputs.at(idx));
      }
      else {
        T* ptr = std::get<T*>(source_id);
        if (!ptr)
          throw std::invalid_argument("source pointer is null");
        sources_.push_back(ptr);
      }
    }

    target_ = &inputs[props.target_id];
  }

  /**
   * Updates the value at target at target as the sum of the values at each source
   */
  void update() {
    *target_ = *sources_[0];
    for (size_t i = 1; i < sources_.size(); i++)
      *target_ += *sources_[i];
  }

private:
  std::vector<T*> sources_{};
  T* target_;
};

}  // namespace teleop

#endif  // TELEOP_CORE_INPUTAGGREGATOR_HPP
