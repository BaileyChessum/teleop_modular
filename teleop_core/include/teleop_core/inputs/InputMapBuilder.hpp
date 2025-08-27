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

#ifndef TELEOP_CORE_INPUTVECTOR_HPP
#define TELEOP_CORE_INPUTVECTOR_HPP

#include <vector>
#include <string>
#include <map>
#include <rclcpp/logging.hpp>
#include <rclcpp/logger.hpp>
#include "teleop_core/inputs/InputAggregator.hpp"
#include "teleop_core/inputs/InputMap.hpp"

namespace teleop
{

template <typename T>
class InputMapBuilder
{
  /**
   * Declares an external T* (or size_t id for internal use) for lookup
   * \name The name of the input, such that the index of the input can be found by looking up this name.
   * \input Either an external T*, or for internal use in push_aggregate, an inputs id for the output InputMap.
   */
  inline void declare_aggregate(const std::string& name, std::variant<size_t, T*> input) {
    const auto it = map_.find(name);
    if (it == map_.end()) {
      // No conflict -- set directly
      map_[name] = input;
      return;
    }

    // Name collision! Resolve through aggregation
    // If there is an existing aggregate, simply append the id to its sources
    const auto aggregate_it = aggregates_.find(name);
    if (aggregate_it != aggregates_.end()) {
      aggregate_it->second.source_ids.push_back(input);
    }
    else {
      // Otherwise make a new aggregate
      const auto aggregate_target_id = push_replace(name);
      typename InputAggregator<T>::Props aggregate{{ it->second, input }, aggregate_target_id };

      aggregates_[name] = aggregate;
    }
  }

  /**
   * Finds the id an input with some name (if it has been defined so far, returns 0 (a safe and valid value) otherwise).
   */
  inline size_t find(const std::string& name) noexcept {
    auto it = map_.find(name);

    if (it == map_.end())
      return 0;
    return it->second;
  }

  /**
   * Requests a new id and corresponding memory on the end of the data vector.
   * \returns The id of the new memory.
   */
  [[nodiscard]] inline size_t push() {
    size_t id = display_names_.size();
    display_names_.emplace_back("(unnamed)");
    return id;
  }

  /**
   * Requests a new id and corresponding memory on the end of the data vector.
   * If there are any name conflicts, the previous name->id entry will be replaced.
   * \name The name of the input, such that the index of the input can be found by looking up this name.
   * \returns The id of the new memory.
   */
  [[nodiscard]] inline size_t push_replace(const std::string& name) {
    size_t id = display_names_.size();
    display_names_.push_back(name);

    // Set id for name
    map_[name] = id;

    return id;
  }

  /**
   * Requests a new id and corresponding memory on the end of the data vector.
   * If there is a conflict, an InputAggregator will be made to combine the two inputs.
   * \name The name of the input, such that the index of the input can be found by looking up this name.
   * \returns The id of the new memory.
   */
  [[nodiscard]] inline size_t push_aggregate(const std::string& name)
  {
    size_t id = display_names_.size();
    display_names_.push_back(name);

    const auto it = map_.find(name);
    if (it == map_.end()) {
      // No conflict -- set directly
      map_[name] = id;
      return id;
    }

    // Name collision! Resolve through aggregation
    // If there is an existing aggregate, simply append the id to its sources
    const auto aggregate_it = aggregates_.find(name);
    if (aggregate_it != aggregates_.end()) {
      aggregate_it->second.source_ids.push_back(id);
    }
    else {
      // Otherwise make a new aggregate
      const auto aggregate_target_id = push_replace(name);
      typename InputAggregator<T>::Props aggregate{{ it->second, id }, aggregate_target_id };

      aggregates_[name] = aggregate;
    }

    return id;
  }

  /**
   * Sets the input with the given id to have the given name
   * \param id The id of the input to rename
   * \param name The new name for the input
   */
//  inline void rename(const size_t id, const std::string& name) noexcept {
//    if (id == 0 || id >= display_names_.size()) {
//      RCLCPP_ERROR(rclcpp::get_logger("input_vector_builder"), "Tried to fetch T* with id (%lu) outside of the range.", id);
//      return;
//    }
//
//    // TODO: Implement
//  }

  /**
   * Gets an input vector with the right size to store all input values in. Use this to crystallize inputs
   */
  [[nodiscard]] InputMap<T> construct() {
    std::vector<typename InputAggregator<T>::Props> aggregates_vector{};
    aggregates_vector.reserve(aggregates_.size());
    for (auto [name, aggregate] : aggregates_)
      aggregates_vector.push_back(aggregate);

    auto inputs = InputMap<T>(display_names_.size(), aggregates_vector, map_);
    return inputs;
  }

private:
  /// Not used for any lookup, but stores the names of inputs, even if they get overridden.
  std::vector<std::string> display_names_{std::string("null")};
  /// Allows the index of some input value in data_ by name
  std::map<std::string, std::variant<size_t, T*>> map_{};

  /// Stores props needed to make aggregates
  std::map<std::string, typename InputAggregator<T>::Props> aggregates_{};
};

}  // namespace teleop

#endif  // TELEOP_CORE_INPUTVECTOR_HPP
