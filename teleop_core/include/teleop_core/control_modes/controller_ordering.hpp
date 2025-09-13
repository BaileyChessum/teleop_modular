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
// Created by Bailey Chessum on 13/9/2025.
//

#ifndef TELEOP_MODULAR_CONTROLLERORDERING_HPP
#define TELEOP_MODULAR_CONTROLLERORDERING_HPP

#include <vector>
#include <set>
#include <string>
#include <map>
#include <math.h>
#include <optional>
#include <rclcpp/logging.hpp>
#include <algorithm>
#include <stdexcept>

namespace teleop {

class ControllerOrdering {
public:
  /**
   * Add an ordered set of controller dependencies
   * \param controllers controller names in the order they need to be activated
   */
  void add(const std::vector<std::string>& controllers)
  {
    if (controllers.size() == 0)
      return;

    auto previous_id = find_or_create_handle(controllers[0]);

    // Make handles for each controller depend on the previous controller in controllers
    for (size_t i = 1; i < controllers.size(); ++i)
    {
      const auto id = find_or_create_handle(controllers[i]);
      handles_[id].dependencies->insert(previous_id);
      handles_[previous_id].children->insert(id);

      previous_id = id;
    }

    is_sorted_ = false;
  }

  /**
   * For a given set of names, gets an ordered set of ids to be activated
   */
  void names_to_ids(const std::vector<std::string> & names, std::set<size_t> & out) {
    out.clear();

    for (const auto& name : names) {
      out.insert((*this)[name]);
    }
  }

  void id_sets_to_names(const std::vector<std::reference_wrapper<std::set<size_t>>>& id_set_refs, std::vector<std::string>& out) {
    out.clear();

    const auto merged_ids = merge_id_sets(id_set_refs);



  }


  /**
   * Merges left and merges right, then gets the differences of the two merged sets from left and right
   * \param[out] left_difference left - right
   * \param[out] right_difference right - left
   */
  static void merged_set_differences(
      const std::vector<std::reference_wrapper<std::set<size_t>>>& left,
      const std::vector<std::reference_wrapper<std::set<size_t>>>& right,
      std::set<size_t>& left_difference,
      std::set<size_t>& right_difference)
  {
    set_differences(merge_id_sets(left), merge_id_sets(right), left_difference, right_difference);
  }

  /**
   * Recursive merges an array of sorted id vectors
   */
  static std::set<size_t> merge_id_sets(const std::vector<std::reference_wrapper<std::set<size_t>>>& id_set_refs) {
    // Copy the references into a vector of sets we can merge
    std::vector<std::set<size_t>> sets;
    for (const auto& ref : id_set_refs) {
      sets.push_back(ref.get());  // make a copy to merge
    }

    while (sets.size() > 1) {
      std::vector<std::set<size_t>> next_round;
      for (size_t i = 0; i + 1 < sets.size(); i += 2) {
        std::set<size_t> merged;
        std::set_union(sets[i].begin(), sets[i].end(),
                       sets[i + 1].begin(), sets[i + 1].end(),
                       std::inserter(merged, merged.begin()));
        next_round.push_back(std::move(merged));
      }
      if (sets.size() % 2 == 1) {
        next_round.push_back(std::move(sets.back()));
      }
      sets = std::move(next_round);
    }

    return sets.empty() ? std::set<size_t>{} : std::move(sets[0]);
  }

  /**
   * Get the index of a name in the well-ordering_
   */
  size_t operator[](const std::string & name) {
    const auto it = name_to_id_.find(name);

    if (it == name_to_id_.end())
      throw std::out_of_range(name + " is not in the controller ordering_.");

    return it->second;
  }

  /**
   * Get the name in the well-ordering_ at the given index
   */
  const std::string& operator[](const size_t id) {
    if (id > handles_.size())
      throw std::out_of_range("Requested controller name larger than the number of elements in the well-ordering_.");

    return handles_[id].name;
  }

  /**
   * Ensures the controllers are sorted
   */
  void sort() {
    if (!is_sorted_)
      order();
  }

private:
  struct Handle
  {
    std::string name;
    std::unique_ptr<std::set<size_t>> dependencies{};
    std::unique_ptr<std::set<size_t>> children{};
  };

  size_t find_or_create_handle(const std::string& controller)
  {
    const auto it = name_to_id_.find(controller);
    if (it != name_to_id_.end())
      return it->second;

    const auto new_id = handles_.size();
    name_to_id_[controller] = new_id;
    handles_.emplace_back(Handle{controller, std::make_unique<std::set<size_t>>(), std::make_unique<std::set<size_t>>()});
    return new_id;
  }

  /**
   * Create value for ordering_
   */
  void order()
  {
    std::vector<size_t> in_degrees{};   //< stores for each handle the number of dependencies of the handle
    in_degrees.reserve(handles_.size());

    std::vector<size_t> ordering{};
    ordering.reserve(handles_.size());

    size_t root_count = 0;

    // Get the in_degree for every handle, and keep track of any root handles
    for (size_t i = 0; i < handles_.size(); ++i)
    {
      in_degrees.emplace_back(handles_[i].dependencies->size());
      if (in_degrees[i] == 0)
      {
        // This element is a root, so we can safely add it to the ordering_
        ordering.emplace_back(i);
        ++root_count;
      }
    }

    // Iterate over every element in the ordering_ to try add its children to the ordering_
    for (size_t i = 0; i < ordering.size(); ++i)
    {
      const auto id = ordering[i];
      const auto& children = handles_[id].children;

      for (const auto& child : *children)
      {
        // decrement the in_degree
        const auto child_in_degree = --in_degrees[child];

        // If we decremented to 0, we can safely add it to the ordering_
        if (child_in_degree == 0)
          ordering.emplace_back(child);
      }
    }

    if (ordering.size() != handles_.size())
    {
      // Uh oh! The graph is not acyclic!
      const auto logger = rclcpp::get_logger("controller_ordering");
      RCLCPP_ERROR(logger,
                   "Failed to create a well-ordering_ for controller activation! You had a cycle in activation order "
                   "that might involve these controller names:");

      // Log the names of all handles_ with an in_degree > 0
      // And incorporate stray handles anyway
      for (size_t j = 0; j < in_degrees.size(); ++j)
      {
        if (in_degrees[j] == 0)
          continue;
        RCLCPP_ERROR(logger, "  - \"%s\"", handles_[j].name.c_str());

        // Incorporate the stray handle
        ordering.emplace_back(j);
      }

      RCLCPP_ERROR(logger,
                   "All controllers need to be specified in the same order for each control mode. You have likely "
                   "changed the order you've listed controllers between different control modes.");
    }

    // Sort handles to match order
    std::vector<size_t> inverse_ordering{};   //< Maps old ids to ordered ids
    inverse_ordering.resize(ordering.size(), 0);

    for (size_t i = 0; i < ordering.size(); ++i) {
      inverse_ordering[ordering[i]] = i;
    }

    // Move handles to match the new ordering_
    std::vector<Handle> new_handles{};
    for (size_t i = 0; i < ordering.size(); ++i) {
      const auto& old_handle = handles_[ordering[i]];

      auto dependencies = std::make_unique<std::set<size_t>>();
      for (const auto dependency : *old_handle.dependencies)
        dependencies->insert(ordering[dependency]);

      auto children = std::make_unique<std::set<size_t>>();
      for (const auto child : *old_handle.children)
        children->insert(ordering[child]);

      new_handles.emplace_back(Handle{old_handle.name, std::move(dependencies), std::move(children)});
    }
    handles_ = std::move(new_handles);

    // Update the ids in the name_to_id_ map to match new ordering_
    for (auto& [_, id] : name_to_id_) {
      id = inverse_ordering[id];
    }

    is_sorted_ = true;
  }

  std::vector<Handle> handles_{};
  std::map<std::string, size_t> name_to_id_{};

  bool is_sorted_ = false;
};

} // teleop

#endif //TELEOP_MODULAR_CONTROLLERORDERING_HPP
