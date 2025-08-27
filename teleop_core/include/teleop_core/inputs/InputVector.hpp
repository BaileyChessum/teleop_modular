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

namespace teleop
{


template <typename T>
class Aggregator


template <typename T>
class InputVector
{
  InputVector() {
  }

  /**
   * Finds the id an input with some name (if it has been defined so far, returns 0 (a safe and valid value) otherwise).
   */
  inline const size_t find(const std::string& name) noexcept {
    auto it = ids_.find(name);

    if (it == ids_.end())
      return 0;
    return it->second;
  }

  /**
   * Requests a new id and corresponding memory on the end of the data vector.
   * \returns The id of the new memory.
   */
  [[nodiscard]] inline const size_t push() {
    size_t id = data_.size();
    data_.emplace_back();
    display_names_.emplace_back("(unnamed)");
    return id;
  }

  /**
   * Requests a new id and corresponding memory on the end of the data vector.
   * \name The name of the input, such that the index of the input can be found by looking up this name.
   * \returns The id of the new memory.
   */
  [[nodiscard]] inline const size_t push(const std::string& name) {
    size_t id = data_.size();
    data_.emplace_back();
    display_names_.push_back(name);

    // Set id for name
    ids_[name] = id;

    return id;
  }

  /**
   * Retrieves an input T* from a given id.
   */
  T* operator[](const size_t id)
  {
    if (id >= data_.size()) {
      RCLCPP_ERROR(rclcpp::get_logger("input_vector"), "Tried to fetch T* with id outside of the range.");
      return &data_[0];
    }

    return &data_[id];
  }

private:
  /// Holds the input values. The first index is populated to act as sink or null value.
  std::vector<T> data_{0};
  /// Not used for any lookup, but stores the names of inputs, even if they get overridden.
  std::vector<std::string> display_names_{std::string("null")};
  /// Allows the index of some input value in data_ by name
  std::map<std::string, size_t> ids_{};

  std::map<std::string, std::vector<size_t>>
};

}  // namespace teleop

#endif  // TELEOP_CORE_INPUTVECTOR_HPP
