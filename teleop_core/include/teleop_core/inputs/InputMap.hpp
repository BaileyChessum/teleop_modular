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

#ifndef CONTROL_MODE_INPUTMAP_HPP
#define CONTROL_MODE_INPUTMAP_HPP

#include <vector>
#include <map>
#include <variant>
#include "teleop_core/inputs/InputAggregator.hpp"

namespace teleop
{

template <typename T>
class InputMap
{
public:
  InputMap(size_t size, const std::vector<typename InputAggregator<T>::Props>& aggregates, const std::map<std::string, std::variant<size_t, T*>>& input_map) {
    inputs_.resize(size, 0);

    // Populate map_, resolving size_t to T*
    for (auto pair : input_map) {
      // Destructuring not used as type inference was poor
      const auto& name = pair.first;
      const std::variant<size_t, T*> input = pair.second;

      if (std::holds_alternative<size_t>(input)) {
        map_[name] = &inputs_[std::get<size_t>(input)];
      }
      else {
        map_[name] = std::get<T*>(input);
      }
    }

    // Construct InputAggregators
    for (auto aggregate : aggregates)
      aggregators_.emplace_back(inputs_, aggregate);
  }

  /**
   * Updates all InputAggregators
   */
  inline void update() {
    for (auto& aggregator : aggregators_)
      aggregator.update();
  }

  using iterator = typename std::map<std::string, T*>::iterator;
  using const_iterator = typename std::map<std::string, T*>::const_iterator;

  constexpr iterator begin() noexcept { return map_.begin(); };
  constexpr iterator end() noexcept { return map_.end(); };
  constexpr const_iterator cbegin() const noexcept { return map_.begin(); }
  constexpr const_iterator cend() const noexcept { return map_.end(); }

  /**
   * Finds the id an input with some name (if it has been defined so far, returns 0 (a safe and valid value) otherwise).
   */
  inline T* find_ptr(const std::string& name) noexcept {
    auto it = map_.find(name);

    if (it == map_.end())
      return nullptr;
    return it->second;
  }

private:
  /// This should never change size
  std::vector<T> inputs_{};
  std::vector<InputAggregator<T>> aggregators_{};
  std::map<std::string, T*> map_{};
};

}  // namespace teleop

#endif  // CONTROL_MODE_INPUTMAP_HPP
