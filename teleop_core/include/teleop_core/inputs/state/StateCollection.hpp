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

#ifndef STATECOLLECTION_HPP
#define STATECOLLECTION_HPP

#include <map>
#include <string>

#include "State.hpp"
#include "teleop_core/inputs/InputManager.hpp"
#include "teleop_core/inputs/InputMapBuilder.hpp"
#include "teleop_core/inputs/input_pipeline_builder.hpp"

namespace teleop::state
{

/**
 * A collection of stateful input declarations, for use for input values created from things other than an input source.
 * @tparam T The type of value held in the state.
 * @tparam InputT The input type wrapping T (Button for bool, Axis for double).
 */
template<typename T, typename InputT>
class StateCollection
{
public:
  explicit StateCollection(InputPipelineElementDelegate& delegate)
    : delegate_(delegate)
  {
  }

  // Delete copy constructor, assignment, and move
  StateCollection(StateCollection && other) = delete;
  StateCollection & operator=(StateCollection && other) = delete;
  StateCollection(const StateCollection &) = delete;
  StateCollection & operator=(const StateCollection &) = delete;

  /**
   * @brief Gets the state for a given key, and returns nullptr when there is no item with that key.
   * @param key The name of the input that the state is registered for.
   * @return a shared pointer to the state with the given key as a name if it exists, nullptr otherwise.
   */
  std::shared_ptr<State<T>> operator[](const std::string & key)
  {
    // Find the element
    auto it = items_.find(key);
    std::shared_ptr<State<T>> ptr = nullptr;

    if (it != items_.end()) {
      ptr = it->second;
    }

    return ptr;
  }

  /**
   * Sets the input with a given name to a given value.
   * @param name name The name of the input to define the value for.
   * @param value value The value to define under that input
   */
  void set(const std::string & name, T value)
  {
    if (std::shared_ptr<State<T>> ptr = (*this)[name]) {
      ptr->value = value;
      return;
    }

    auto logger = rclcpp::get_logger("state_collection");

    RCLCPP_DEBUG(logger, "Making new state shared pointer.");
    // Make and register a new state
    const auto state = std::make_shared<State<T>>(name, value);
    state->value = value;
    items_[name] = state;

    RCLCPP_DEBUG(logger, "Relinking input pipeline.");
    delegate_.relink();
  }

  /**
   * Remove the input definition with the given name.
   * @param name The name of the input to clear.
   */
  void clear(const std::string & name)
  {
    // Find the element
    auto it = items_.find(name);

    if (it == items_.end()) {
      // Do nothing, as its already undefined
      return;
    }

    auto shared_ptr = it->second;

    auto logger = rclcpp::get_logger("state_collection");
    RCLCPP_DEBUG(logger, "Erasing \"%s\" from collection, then relinking", name.c_str());
    items_.erase(name);
    delegate_.relink();

    RCLCPP_DEBUG(logger, "Dropping state shared pointer for \"%s\"", name.c_str());
    shared_ptr.reset();
  }

  // Make the collection iterable
  using iterator = typename std::map<std::string, typename State<T>::SharedPtr>::iterator;
  using const_iterator = typename std::map<std::string, typename State<T>::SharedPtr>::const_iterator;
  iterator begin() { return items_.begin(); }
  const_iterator cbegin() { return items_.cbegin(); }
  iterator end() { return items_.end(); }
  const_iterator cend() { return items_.cend(); }

private:
  std::map<std::string, typename State<T>::SharedPtr> items_{};
  InputPipelineElementDelegate& delegate_;
};

}  // namespace teleop::state

#endif  // STATECOLLECTION_HPP
