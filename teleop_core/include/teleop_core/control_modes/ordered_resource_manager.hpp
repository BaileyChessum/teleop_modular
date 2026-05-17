// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#ifndef TELEOP_CORE__CONTROL_MODES__ORDERED_RESOURCE_MANAGER_HPP_
#define TELEOP_CORE__CONTROL_MODES__ORDERED_RESOURCE_MANAGER_HPP_

#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/node.hpp>

#include "teleop_core/control_modes/controller_ordering.hpp"

namespace teleop
{

class OrderedResourceManager
{
public:
  enum class RefreshResult
  {
    Failed,
    Unchanged,
    Changed,
  };

  OrderedResourceManager(rclcpp::Node::SharedPtr node, std::string logger_child_name);
  virtual ~OrderedResourceManager();

  void activate_for_modes(const std::vector<size_t> & activate_cm_ids);
  void deactivate_for_modes(const std::vector<size_t> & deactivate_cm_ids);
  void switch_active(
    const std::vector<size_t> & deactivate_cm_ids,
    const std::vector<size_t> & activate_cm_ids);
  void sync_to_active_modes(const std::vector<size_t> & active_cm_ids);

  static void set_differences(
    const std::set<size_t> & left,
    const std::set<size_t> & right,
    std::set<size_t> & left_difference,
    std::set<size_t> & right_difference);

protected:
  struct Context
  {
    explicit Context(rclcpp::Node::SharedPtr node_in, rclcpp::Logger logger_in)
    : node(std::move(node_in)), logger(std::move(logger_in))
    {
    }

    rclcpp::Node::SharedPtr node;
    ControllerOrdering ordering{};
    std::vector<std::set<size_t>> resources_for_ids{};
    std::set<size_t> current_active_resources{};
    std::set<size_t> desired_active_resources{};
    std::mutex state_mutex{};
    rclcpp::Logger logger;
  };

  bool register_resources_for_ids(
    const std::vector<std::reference_wrapper<const std::vector<std::string>>> & resource_names_for_ids);

  [[nodiscard]] Context & context() noexcept
  {
    return *context_;
  }

  [[nodiscard]] const Context & context() const noexcept
  {
    return *context_;
  }

  [[nodiscard]] bool stop_requested() const noexcept;

  template<typename TRep, typename TPeriod>
  bool wait_for_stop_or(const std::chrono::duration<TRep, TPeriod> & wait_time)
  {
    std::unique_lock lock(update_mutex_);
    return update_condition_.wait_for(lock, wait_time, [this]() {return stop_requested_;});
  }

  template<typename FutureAndRequestId>
  auto wait_for_response(
    std::optional<FutureAndRequestId> & future,
    const char * description,
    bool impatient,
    double reasonable_timeout) -> decltype(std::declval<FutureAndRequestId &>().get())
  {
    using ResultType = decltype(std::declval<FutureAndRequestId &>().get());

    if (!(future.has_value() && future->valid())) {
      return ResultType{};
    }

    const auto read_result = [&](auto & pending_future) -> ResultType {
        try {
          auto response = pending_future.get();
          future = std::nullopt;
          return response;
        } catch (const std::exception & e) {
          RCLCPP_ERROR(context().logger, "%s service call failed: %s", description, e.what());
          future = std::nullopt;
          return ResultType{};
        }
      };

    if (!impatient) {
      auto status = future->wait_for(std::chrono::duration<double>(reasonable_timeout));
      if (status == std::future_status::ready) {
        return read_result(*future);
      }
    }

    while (!stop_requested() && future.has_value() && future->valid()) {
      auto status = future->wait_for(std::chrono::milliseconds(100));
      if (status != std::future_status::ready) {
        continue;
      }

      return read_result(*future);
    }

    return ResultType{};
  }

  RefreshResult replace_current_active_resources(std::set<size_t> actual_active_resources);
  void start_worker();

  [[nodiscard]] virtual bool should_log_resources() const = 0;
  [[nodiscard]] virtual const char * resource_kind_plural() const = 0;
  virtual bool apply_resource_change(
    const std::vector<std::string> & resource_names_to_deactivate,
    const std::vector<std::string> & resource_names_to_activate) = 0;
  virtual RefreshResult refresh_current_active_resources() = 0;

private:
  void notify_update();
  void mutate_desired_active_resources(
    const std::vector<size_t> & deactivate_cm_ids,
    const std::vector<size_t> & activate_cm_ids);
  void worker_main();
  void process_pending_updates();

  std::shared_ptr<Context> context_;

  std::thread worker_;
  mutable std::mutex update_mutex_{};
  std::condition_variable update_condition_{};
  bool stop_requested_ = false;
  bool update_pending_ = false;
};

}  // namespace teleop

#endif  // TELEOP_CORE__CONTROL_MODES__ORDERED_RESOURCE_MANAGER_HPP_
