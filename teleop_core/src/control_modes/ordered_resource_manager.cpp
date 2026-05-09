// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#include "teleop_core/control_modes/ordered_resource_manager.hpp"

#include <algorithm>

namespace teleop
{

OrderedResourceManager::OrderedResourceManager(
  rclcpp::Node::SharedPtr node,
  std::string logger_child_name)
: context_(std::make_shared<Context>(
    node,
    node->get_logger().get_child(logger_child_name)))
{
}

OrderedResourceManager::~OrderedResourceManager()
{
  {
    std::lock_guard lock(update_mutex_);
    stop_requested_ = true;
    update_pending_ = true;
  }
  update_condition_.notify_all();

  if (worker_.joinable()) {
    worker_.join();
  }
}

bool OrderedResourceManager::register_resources_for_ids(
  const std::vector<std::reference_wrapper<const std::vector<std::string>>> & resource_names_for_ids)
{
  std::lock_guard lock(context_->state_mutex);

  context_->ordering.clear();
  context_->resources_for_ids.clear();
  context_->current_active_resources.clear();
  context_->desired_active_resources.clear();

  for (const auto & names_ref : resource_names_for_ids) {
    context_->ordering.add(names_ref.get());
  }

  if (!context_->ordering.sort()) {
    return false;
  }

  context_->resources_for_ids.reserve(resource_names_for_ids.size());
  for (const auto & names_ref : resource_names_for_ids) {
    std::set<size_t> ids{};
    context_->ordering.names_to_ids(names_ref.get(), ids);
    context_->resources_for_ids.emplace_back(std::move(ids));
  }

  if (should_log_resources()) {
    RCLCPP_INFO(context_->logger, "Created %s activation order:", resource_kind_plural());
    for (size_t i = 0; i < context_->ordering.size(); ++i) {
      RCLCPP_INFO(context_->logger, "  - \"%s\"", context_->ordering[i].c_str());
    }
    RCLCPP_INFO(context_->logger, " ");

    RCLCPP_INFO(context_->logger, "%s for each control mode:", resource_kind_plural());
    for (size_t i = 0; i < context_->resources_for_ids.size(); ++i) {
      RCLCPP_INFO(context_->logger, "  [%lu]", i);
      for (const auto id : context_->resources_for_ids[i]) {
        RCLCPP_INFO(context_->logger, "    - \"%s\" (id: %lu)", context_->ordering[id].c_str(), id);
      }
    }
    RCLCPP_INFO(context_->logger, " ");
  }

  return true;
}

void OrderedResourceManager::activate_for_modes(const std::vector<size_t> & activate_cm_ids)
{
  mutate_desired_active_resources({}, activate_cm_ids);
  notify_update();
}

void OrderedResourceManager::deactivate_for_modes(const std::vector<size_t> & deactivate_cm_ids)
{
  mutate_desired_active_resources(deactivate_cm_ids, {});
  notify_update();
}

void OrderedResourceManager::switch_active(
  const std::vector<size_t> & deactivate_cm_ids,
  const std::vector<size_t> & activate_cm_ids)
{
  mutate_desired_active_resources(deactivate_cm_ids, activate_cm_ids);
  notify_update();
}

void OrderedResourceManager::sync_to_active_modes(const std::vector<size_t> & active_cm_ids)
{
  std::vector<std::reference_wrapper<std::set<size_t>>> active_id_sets{};
  active_id_sets.reserve(active_cm_ids.size());

  {
    std::lock_guard lock(context_->state_mutex);
    for (const auto cm_id : active_cm_ids) {
      active_id_sets.emplace_back(std::ref(context_->resources_for_ids.at(cm_id)));
    }

    context_->desired_active_resources = ControllerOrdering::merge_id_sets(active_id_sets);
  }

  notify_update();
}

void OrderedResourceManager::set_differences(
  const std::set<size_t> & left,
  const std::set<size_t> & right,
  std::set<size_t> & left_difference,
  std::set<size_t> & right_difference)
{
  left_difference.clear();
  right_difference.clear();

  auto l_it = left.begin();
  auto r_it = right.begin();

  while (l_it != left.end() && r_it != right.end()) {
    if (*l_it < *r_it) {
      left_difference.insert(*l_it);
      ++l_it;
    } else if (*l_it > *r_it) {
      right_difference.insert(*r_it);
      ++r_it;
    } else {
      ++l_it;
      ++r_it;
    }
  }

  left_difference.insert(l_it, left.end());
  right_difference.insert(r_it, right.end());
}

bool OrderedResourceManager::stop_requested() const noexcept
{
  std::lock_guard lock(update_mutex_);
  return stop_requested_;
}

OrderedResourceManager::RefreshResult OrderedResourceManager::replace_current_active_resources(
  std::set<size_t> actual_active_resources)
{
  std::lock_guard lock(context_->state_mutex);

  if (context_->current_active_resources == actual_active_resources) {
    return RefreshResult::Unchanged;
  }

  context_->current_active_resources = std::move(actual_active_resources);
  return RefreshResult::Changed;
}

void OrderedResourceManager::start_worker()
{
  if (!worker_.joinable()) {
    worker_ = std::thread([this]() {worker_main();});
  }
}

void OrderedResourceManager::notify_update()
{
  {
    std::lock_guard lock(update_mutex_);
    update_pending_ = true;
  }
  update_condition_.notify_one();
}

void OrderedResourceManager::mutate_desired_active_resources(
  const std::vector<size_t> & deactivate_cm_ids,
  const std::vector<size_t> & activate_cm_ids)
{
  std::vector<std::reference_wrapper<std::set<size_t>>> deactivate_id_sets{};
  std::vector<std::reference_wrapper<std::set<size_t>>> activate_id_sets{};

  deactivate_id_sets.reserve(deactivate_cm_ids.size());
  activate_id_sets.reserve(activate_cm_ids.size());

  std::lock_guard lock(context_->state_mutex);

  for (const auto cm_id : deactivate_cm_ids) {
    deactivate_id_sets.emplace_back(std::ref(context_->resources_for_ids.at(cm_id)));
  }
  for (const auto cm_id : activate_cm_ids) {
    activate_id_sets.emplace_back(std::ref(context_->resources_for_ids.at(cm_id)));
  }

  const auto deactivate_ids = ControllerOrdering::merge_id_sets(deactivate_id_sets);
  const auto activate_ids = ControllerOrdering::merge_id_sets(activate_id_sets);

  for (const auto id : deactivate_ids) {
    context_->desired_active_resources.erase(id);
  }
  for (const auto id : activate_ids) {
    context_->desired_active_resources.insert(id);
  }
}

void OrderedResourceManager::worker_main()
{
  RCLCPP_DEBUG(context_->logger, "Starting %s worker loop.", resource_kind_plural());

  while (true) {
    {
      std::unique_lock lock(update_mutex_);
      update_condition_.wait(lock, [this]() {return update_pending_ || stop_requested_;});
      if (stop_requested_) {
        break;
      }
      update_pending_ = false;
    }

    process_pending_updates();
  }

  RCLCPP_DEBUG(context_->logger, "Stopping %s worker loop.", resource_kind_plural());
}

void OrderedResourceManager::process_pending_updates()
{
  const size_t max_iterations = std::max<size_t>(4, context_->ordering.size() * 2 + 2);

  for (size_t iteration = 0; iteration < max_iterations; ++iteration) {
    if (stop_requested()) {
      return;
    }

    std::set<size_t> current_active_resources{};
    std::set<size_t> desired_active_resources{};
    {
      std::lock_guard lock(context_->state_mutex);
      current_active_resources = context_->current_active_resources;
      desired_active_resources = context_->desired_active_resources;
    }

    std::set<size_t> deactivate_set{};
    std::set<size_t> activate_set{};
    set_differences(current_active_resources, desired_active_resources, deactivate_set, activate_set);

    if (deactivate_set.empty() && activate_set.empty()) {
      if (desired_active_resources.empty() && current_active_resources.empty()) {
        return;
      }

      const auto refresh_result = refresh_current_active_resources();
      if (refresh_result != RefreshResult::Changed) {
        return;
      }
      continue;
    }

    if (should_log_resources()) {
      RCLCPP_INFO(context_->logger, "desired_active_%s:", resource_kind_plural());
      for (const auto id : desired_active_resources) {
        RCLCPP_INFO(context_->logger, "    - \"%s\" (id: %lu)", context_->ordering[id].c_str(), id);
      }
      RCLCPP_INFO(context_->logger, "current_active_%s:", resource_kind_plural());
      for (const auto id : current_active_resources) {
        RCLCPP_INFO(context_->logger, "    - \"%s\" (id: %lu)", context_->ordering[id].c_str(), id);
      }
      RCLCPP_INFO(context_->logger, " ");
    }

    std::vector<std::string> deactivate_names{};
    std::vector<std::string> activate_names{};
    deactivate_names.reserve(deactivate_set.size());
    activate_names.reserve(activate_set.size());

    for (auto it = deactivate_set.rbegin(); it != deactivate_set.rend(); ++it) {
      deactivate_names.emplace_back(context_->ordering[*it]);
    }
    for (const auto id : activate_set) {
      activate_names.emplace_back(context_->ordering[id]);
    }

    if (!apply_resource_change(deactivate_names, activate_names)) {
      return;
    }

    const auto refresh_result = refresh_current_active_resources();
    if (refresh_result != RefreshResult::Changed) {
      return;
    }
  }

  RCLCPP_WARN(
    context_->logger,
    "Stopping %s reconciliation after too many iterations. The external system may still be out of sync.",
    resource_kind_plural());
}

}  // namespace teleop
