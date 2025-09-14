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

#ifndef CONTROL_MODE_CONTROLLER_MANAGER_MANAGER_HPP
#define CONTROL_MODE_CONTROLLER_MANAGER_MANAGER_HPP

#include <set>
#include <utility>
#include <vector>
#include <string>
#include "controller_ordering.hpp"
#include <rclcpp/node.hpp>
#include <thread>
#include <controller_manager_msgs/srv/switch_controller.hpp>

namespace teleop
{

/**
 * Helper class for managing the ros2_control controller manager.
 * Spins up its own thread to manage communication with the controller manager.
 */
class ControllerManagerManager
{
public:
  explicit ControllerManagerManager(rclcpp::Node::SharedPtr node)
    : node_(std::move(node))
  {
  }

  ~ControllerManagerManager() {
    if (worker_) {
      worker_->end_loop();
      worker_ = nullptr;
    }
  }

  /**
   * For each control mode, declares what the controller names are. This allows the ordering to be calculated.
   * The service loop thread_ is started as a side effect.
   */
  void register_controllers_for_ids(const std::vector<std::reference_wrapper<const std::vector<std::string>>> & controller_names_for_ids)
  {
    if (worker_) {
      worker_->end_loop();
      worker_ = nullptr;
    }

    for (const auto& names_ref : controller_names_for_ids)
      context_->ordering_.add(names_ref.get());
    context_->ordering_.sort();

    context_->controllers_for_ids_.clear();
    for (const auto& names_ref : controller_names_for_ids)
    {
      std::set<size_t> ids{};
      context_->ordering_.names_to_ids(names_ref.get(), ids);

      context_->controllers_for_ids_.emplace_back(std::move(ids));
    }

    RCLCPP_INFO(context_->logger, "Created controller order:");
    for (size_t i = 0; i < context_->ordering_.size(); ++i) {
      RCLCPP_INFO(context_->logger, "  - \"%s\"", context_->ordering_[i].c_str());
    }

    RCLCPP_INFO(context_->logger, "Control mode controllers:");
    for (size_t i = 0; i < context_->controllers_for_ids_.size(); ++i) {
      RCLCPP_INFO(context_->logger, "  [%lu]", i);
      for (const auto id : context_->controllers_for_ids_[i])
      {
        RCLCPP_INFO(context_->logger, "    - \"%s\" (id: %lu)", context_->ordering_[id].c_str(), id);
      }
    }

  }

  /**
   * Tries to update the active controllers, given control mode ids to be activated and deactivated.
   * \param activate_cm_ids The set of control mode ids being activated
   * \param deactivate_cm_ids The set of control mode ids being deactivated
   */
  void switch_active(const std::vector<size_t> & deactivate_cm_ids, const std::vector<size_t> & activate_cm_ids) {
    mutate_desired_active_controllers(deactivate_cm_ids, activate_cm_ids);
    invoke_update();
  }

  /**
   * Tries to get a worker thread to call the controller manager
   */
  void invoke_update() {
    // TODO: Spawn or reuse thread to perform update here

    if (worker_ && !worker_->running())
      worker_ = nullptr;  //< Should never run, but added for safety

    if (worker_) {
      if (worker_->can_invoke_update()) {
        // Reuse existing worker thread
        worker_->invoke_update();
        return;
      }

      // Kill the existing worker thread
      worker_->end_loop();
      worker_ = nullptr;
    }

    // Start a new worker thread
    worker_ = std::make_shared<Worker>(context_);
    worker_->start();
    worker_->invoke_update();
  }

  /**
   * Gets the differences of two sets left and right
   * \param[out] left_difference left - right
   * \param[out] right_difference right - left
   */
  static void set_differences(
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
      }
      else if (*l_it > *r_it) {
        right_difference.insert(*r_it);
        ++r_it;
      }
      else {
        // element is common to both sets, skip both
        ++l_it;
        ++r_it;
      }
    }

    // Finish inserting elements for the other set when we finish inserting elements for one set
    left_difference.insert(l_it, left.end());
    right_difference.insert(r_it, right.end());
  }

private:
  /**
   * Contains data shared between the class and any Workers
   */
  struct Context {
    explicit Context(const rclcpp::Node::SharedPtr & node);

    rclcpp::Node::SharedPtr node;
    /// Client to call the service on the controller manager to change the currently active controllers.
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_ = nullptr;

    std::vector<std::set<size_t>> controllers_for_ids_{};  //< set of controller ids for each control mode id
    ControllerOrdering ordering_{};

    /// The set of controllers we think are actually active on the controller manager
    std::set<size_t> current_active_controllers_{};
    /// Mutex to guard current_active_controllers_ changes
    std::mutex current_active_controllers_mutex_{};

    /// The set of controllers we want to be active on the controller manager
    std::set<size_t> desired_active_controllers_{};
    /// Mutex to guard desired_active_controllers_ changes
    std::mutex desired_active_controllers_mutex_{};

    rclcpp::Logger logger;
  };

  /**
   * Class that manages the lifecycle of a worker thread. This was necessary as getting the result of a service call is
   * blocking, and thus needs a whole thread to manage.
   */
  class Worker : public std::enable_shared_from_this<Worker> {
  public:
    explicit Worker(std::shared_ptr<Context> context) : context_(std::move(context)) {
    }

    ~Worker() {
      if (thread_.joinable()) {
        RCLCPP_ERROR(context_->logger, "Tried to destruct a worker thread without joining the running thread.");
        thread_.detach();
      }
      thread_ = {};
    }

    /**
     * Starts the worker thread. After calling, the worker will keep itself alive.
     */
    void start() {
      if (running_.load())
        return;
      running_ = true;
      auto self = shared_from_this();

      // Lambda captures the shared pointer so this object stays alive while the thread runs
      thread_ = std::thread([self]() {
        self->main();
      });
    }

    /**
     * Make the thread stop without waiting for the thread to stop.
     */
    void end_loop() {
      running_.store(false, std::memory_order_release);
      update_condition_.notify_all();

      thread_.detach();
    }

    /**
     * Returns true if this worker can accept a new update
     */
    [[nodiscard]] inline bool can_invoke_update() const noexcept {
      return can_invoke_update_.load(std::memory_order_acquire);
    }

    /**
     * Returns true if this worker is runnign
     */
    [[nodiscard]] inline bool running() const noexcept {
      return running_.load(std::memory_order_acquire);
    }

    /**
     * Unblocks the service thread to send a new message to the controller manager.
     */
    void invoke_update() {
      if (!can_invoke_update()) {
        RCLCPP_ERROR(context_->logger, "Tried to invoke an update, but can_invoke_update() is false.");
        return;
      }

      std::lock_guard lock(mutex_); //< Just to be safe with needing to reset after waiting

      should_update_.store(true, std::memory_order_release);
      update_condition_.notify_one();
    }

  private:
    /**
     * Entry point the the worker thread.
     */
    void main() {
      RCLCPP_DEBUG(context_->logger, "Starting service loop.");

      while (running_.load()) {
        wait_for_change();

        if (!running_.load())
          break;

        update_controllers();

        if (!running_.load())
          break;

        wait_for_future();
      }

      RCLCPP_DEBUG(context_->logger, "Service loop ending.");
    }

    /**
     * Blocks until desired_active_controllers_ changes (should_update_ becomes true),
     * or the thread should stop running (running_ becomes false)
     */
    void wait_for_change() {
      std::unique_lock lock(mutex_);
      can_invoke_update_.store(true, std::memory_order_release);

      // Wait for changes to desired_active_controllers_
      update_condition_.wait(
          lock, [&] {
            return should_update_.load(std::memory_order_acquire) || !running_.load(std::memory_order_acquire);
          });

      should_update_.store(false, std::memory_order_release);
      can_invoke_update_.store(false, std::memory_order_release);
    }

    /**
     * Waits for a given amount of time, or until the thread needs to exit
     */
    template<typename TRep, typename TPeriod>
    bool wait_for(const std::chrono::duration<TRep, TPeriod> & wait_time) {
      std::unique_lock lock(mutex_);

      // Wait for changes to desired_active_controllers_
      auto result = update_condition_.wait_for(lock, wait_time, [&] {
        return !running_.load(std::memory_order_acquire);
      });

      return result;
    }

    /**
     * Waits until the future is ready.
     * Polls running_ every 100ms to try stop
     */
    void wait_for_future() {
      // Wait for any switch controllers future to complete
      while (running_.load() && future_.has_value() && future_->valid()) {
        // TODO: Parameterize period
        auto status = future_->wait_for(std::chrono::milliseconds(100));

        if (status != std::future_status::ready)
          continue;

        try {
          auto result = future_->get(); // consume future
          service_future_result(result);
        } catch (const std::exception& e) {
           RCLCPP_ERROR(context_->logger, "Service call failed: %s", e.what());
        }
        break;  // done with this request
      }
    }

    /**
     * Calculates the difference between current_active_controllers and desired_active_controllers,
     * then repeatedly calls switch_controllers()
     */
    void update_controllers() {
      std::set<size_t> desired_active_controllers;
      std::set<size_t> current_active_controllers;

      std::set<size_t> deactivate_set{};
      std::set<size_t> activate_set{};

      { // Copy current controls while holding a lock
        std::lock_guard lock(context_->current_active_controllers_mutex_);
        current_active_controllers = context_->current_active_controllers_;
      }

      { // Copy active controls while holding a lock
        std::lock_guard lock(context_->desired_active_controllers_mutex_);
        desired_active_controllers = context_->desired_active_controllers_;
      }

      RCLCPP_INFO(context_->logger, "desired_active_controllers_:");
      for (const auto id : desired_active_controllers)
        RCLCPP_INFO(context_->logger, "    - \"%s\" (id: %lu)", context_->ordering_[id].c_str(), id);
      RCLCPP_INFO(context_->logger, "current_active_controllers_:");
      for (const auto id : current_active_controllers)
        RCLCPP_INFO(context_->logger, "    - \"%s\" (id: %lu)", context_->ordering_[id].c_str(), id);

      set_differences(current_active_controllers, desired_active_controllers, deactivate_set, activate_set);

      RCLCPP_INFO(context_->logger, "deactivate_set:");
      for (const auto id : deactivate_set)
        RCLCPP_INFO(context_->logger, "    - \"%s\" (id: %lu)", context_->ordering_[id].c_str(), id);
      RCLCPP_INFO(context_->logger, "activate_set:");
      for (const auto id : activate_set)
        RCLCPP_INFO(context_->logger, "    - \"%s\" (id: %lu)", context_->ordering_[id].c_str(), id);

      // Convert the id sets into controller names
      std::vector<std::string> deactivate_names{};
      std::vector<std::string> activate_names{};

      deactivate_names.reserve(deactivate_set.size());
      activate_names.reserve(activate_set.size());

      // deactivate names are provided in reverse order
      for (auto it = deactivate_set.rbegin(); it != deactivate_set.rend(); ++it)
        deactivate_names.emplace_back(context_->ordering_[*it]);

      for (auto id : activate_set)
        activate_names.emplace_back(context_->ordering_[id]);

      if (!running())
        return;

      // Attempt the switch
      bool result = switch_controllers(deactivate_names, activate_names);

      // Repeatedly retry a fixed number of times, with a short delay between attempts
      const int max_attempts = 10;
      // TODO: Parameterize retry count
      for (int i = 1; i < max_attempts && !result && running_.load(); ++i) {
        // Wait for a short amount of time, or until the thread should stop running.
        // TODO: Parameterize period
        wait_for(std::chrono::milliseconds(100));
        if (!running_.load())
          return;

        result = switch_controllers(deactivate_names, activate_names);
      }

      if (running() && !result) {
        RCLCPP_ERROR(context_->logger, "/controller_manager/switch_controller not available after %d attempts. Stopping attempts.",
                     max_attempts);
      }
    }

    /**
     * Makes the call to the controller manager to try switch the active controllers
     * \returns True if the controller manager service was available and the service was called.
     */
    bool switch_controllers(
        const std::vector<std::string> & controllers_to_deactivate,
        const std::vector<std::string> & controllers_to_activate)
    {
      RCLCPP_INFO(context_->logger, "Current active controllers:");
      for (const auto id : context_->current_active_controllers_) {
        const auto name = context_->ordering_[id];
        RCLCPP_INFO(context_->logger, "  - %s", name.c_str());
      }

      RCLCPP_INFO(context_->logger, "Deactivating:");
      for (const auto& name : controllers_to_deactivate) {
        RCLCPP_INFO(context_->logger, "  - %s", name.c_str());
      }

      RCLCPP_INFO(context_->logger, "Activating:");
      for (const auto& name : controllers_to_activate) {
        RCLCPP_INFO(context_->logger, "  - %s", name.c_str());
      }

      if (controllers_to_deactivate.empty() && controllers_to_activate.empty()) {
        return true;
      }

      if (!context_->switch_controller_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(context_->logger, *context_->node->get_clock(), 2000, "/controller_manager/switch_controller service not currently available.");
        return false;
      }

      const auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
      request->deactivate_controllers = controllers_to_deactivate;
      request->activate_controllers = controllers_to_activate;
      request->strictness = 2;
      request->activate_asap = true;


      future_ = context_->switch_controller_client_->async_send_request(request);
      return true;
    }

    /**
     * Called when we get the result back from a switch controller request
     */
    void service_future_result(const controller_manager_msgs::srv::SwitchController::Response::SharedPtr & response) {
      if (!response->ok) {
        on_switch_failure();
        return;
      }

      // We can now assume context_->current_active_controllers_ represents the actual state of the controllers
      {
        // TODO: Confirm we wont cause deadlock
        std::lock_guard lock2(context_->current_active_controllers_mutex_);
        std::lock_guard lock1(context_->desired_active_controllers_mutex_);

        context_->current_active_controllers_ = context_->desired_active_controllers_;
      }

      RCLCPP_INFO(context_->logger, "Successfully switched controllers.");

      // TODO: We could add a sanity check call to
      //  /controller_manager/list_controllers : controller_manager_msgs/srv/ListControllers to double check
    }

    /**
     * Called whenever a switch request finished without succeeding.
     */
    void on_switch_failure() {
      RCLCPP_ERROR(context_->logger, "Failed to switch controllers.");

      // TODO: Get current active controllers by calling /controller_manager/list_controllers : controller_manager_msgs/srv/ListControllers

      // TODO: Warn of any irreconcilable differences in desired state. Did an important controller fail?

      // TODO: Load the current active controllers into context_->current_active_controllers_

      // TODO: Load any missing controllers, with warning

      // TODO: Perform switch request to make the controller

    }

    std::shared_ptr<Context> context_;
    std::thread thread_;

    // When true, the thread will try its best to exit and die
    std::atomic<bool> running_ = false;
    // When true, you can request updates from this thread. Otherwise, you need to spawn a new thread and kill this one.
    std::atomic<bool> can_invoke_update_ = true;

    // Concurrency control
    /// Mutex to guard update condition changes
    std::mutex mutex_;
    /// Lock that waits until should_update_ is true.
    std::condition_variable update_condition_;
    std::atomic<bool> should_update_ = false;

    // Service calls
    /// Any previous result from a service call
    std::optional<rclcpp::Client<controller_manager_msgs::srv::SwitchController>::FutureAndRequestId> future_;
  };

  /**
   * Holds a mutex on desired_active_controllers_, then removes controller ids associated with each deactivate_cm_ids
   * entry, and adds controller ids associated with each activate_cm_ids entry.
   */
  void mutate_desired_active_controllers(const std::vector<size_t> & deactivate_cm_ids, const std::vector<size_t> & activate_cm_ids) {
    // Get deactivate ids
    std::vector<std::reference_wrapper<std::set<size_t>>> deactivate_id_sets{};
    deactivate_id_sets.reserve(deactivate_cm_ids.size());
    for (const auto cm_id : deactivate_cm_ids)
      deactivate_id_sets.emplace_back(std::ref(context_->controllers_for_ids_[cm_id]));
    const auto deactivate_ids = ControllerOrdering::merge_id_sets(deactivate_id_sets);

    // Get activate ids
    std::vector<std::reference_wrapper<std::set<size_t>>> activate_id_sets{};
    activate_id_sets.reserve(activate_cm_ids.size());
    for (const auto cm_id : activate_cm_ids)
      activate_id_sets.emplace_back(std::ref(context_->controllers_for_ids_[cm_id]));
    const auto activate_ids = ControllerOrdering::merge_id_sets(activate_id_sets);

    // Hold lock
    std::lock_guard lock(context_->desired_active_controllers_mutex_);

    // Remove from desired ids
    for (const auto id : deactivate_ids)
      context_->desired_active_controllers_.erase(id);

    // Add to desired ids
    for (const auto id : activate_ids)
      context_->desired_active_controllers_.insert(id);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Context> context_ = std::make_shared<Context>(node_);
  std::shared_ptr<Worker> worker_ = nullptr;
};

}  // namespace teleop

#endif  // CONTROL_MODE_CONTROLLER_MANAGER_MANAGER_HPP
