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
#include <controller_manager_msgs/srv/list_controllers.hpp>

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
      context_->ordering.add(names_ref.get());
    context_->ordering.sort();

    context_->controllers_for_ids.clear();
    for (const auto& names_ref : controller_names_for_ids)
    {
      std::set<size_t> ids{};
      context_->ordering.names_to_ids(names_ref.get(), ids);

      context_->controllers_for_ids.emplace_back(std::move(ids));
    }

    if (context_->params.log_controllers) {
      RCLCPP_INFO(context_->logger, "Created controller activation order:");
      for (size_t i = 0; i < context_->ordering.size(); ++i) {
        RCLCPP_INFO(context_->logger, "  - \"%s\"", context_->ordering[i].c_str());
      }
      RCLCPP_INFO(context_->logger, " ");

      RCLCPP_INFO(context_->logger, "Controllers for each control mode:");
      for (size_t i = 0; i < context_->controllers_for_ids.size(); ++i) {
        RCLCPP_INFO(context_->logger, "  [%lu]", i);
        for (const auto id : context_->controllers_for_ids[i])
          RCLCPP_INFO(context_->logger, "    - \"%s\" (id: %lu)", context_->ordering[id].c_str(), id);
      }
      RCLCPP_INFO(context_->logger, " ");
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
    /**
     * Parameters used by the controller manager manager
     */
    struct Params {
      /// The name of the controller manager to use
      std::string controller_manager = "/controller_manager";
      /// Whether to log info about controller switching
      bool log_controllers = false;

      /// Whether to wait (for the specified reasonable time) for previous requests to finish before starting new
      /// switch requests
      bool impatient = false;

      // switch_controller service message values:
      int strictness = 2;
      bool activate_asap = true;
      builtin_interfaces::msg::Duration timeout{};

      /// Time in seconds to keep waiting for a previous switch request to finish before giving up on it "
      /// and making a new switch request
      double reasonable_timeout = 3.0;

      int connection_retry_count = 10;
      double connection_retry_rate = 10.0;
    };

    /**
     * Constructor. Creates service clients and gets param values.
     */
    explicit Context(const rclcpp::Node::SharedPtr & node);

    /// The main teleop_node ROS2 node
    rclcpp::Node::SharedPtr node;
    /// Client to call the service on the controller manager to change the currently active controllers.
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client = nullptr;
    /// Client to call the service on the controller manager to change the currently active controllers.
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client = nullptr;

    /// ROS2 node parameters used for configuration of controller manager manager behaviour
    Params params{};

    /// Contains the set of controller ids to activate for each control mode id
    std::vector<std::set<size_t>> controllers_for_ids{};  //< set of controller ids for each control mode id
    /// Contains the controller activation order, and controller id <-> name conversions
    ControllerOrdering ordering{};

    /// The set of controllers we think are actually active on the controller manager
    std::set<size_t> current_active_controllers{};
    /// Mutex to guard current_active_controllers_ changes
    std::mutex current_active_controllers_mutex{};

    /// The set of controllers we want to be active on the controller manager
    std::set<size_t> desired_active_controllers{};
    /// Mutex to guard desired_active_controllers_ changes
    std::mutex desired_active_controllers_mutex{};

    /// Logger used by all threads. Same as logger for the teleop_node, suffixed with .controller_manager_manager
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

      while (running()) {
        wait_for_change();

        if (!running())
          break;

        update_controllers();

        if (!running())
          break;

        // Don't do sanity check if theres nothing in ros2_control
        if (!context_->desired_active_controllers.empty() || !context_->current_active_controllers.empty()) {
          // Check the actual active controllers in ros2_control
          if (get_actual_current_controllers()) {
            if (!running())
              break;

            // Something changed as a result of checking ros2_control, so perform another update on the controllers
            update_controllers();
          }
        }
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
    controller_manager_msgs::srv::SwitchController::Response::SharedPtr wait_for_future() {
      controller_manager_msgs::srv::SwitchController::Response::SharedPtr result = nullptr;

      if (!(running_.load() && future_.has_value() && future_->valid()))
        return nullptr;

      // Do initial patient waiting
      if (!context_->params.impatient)
      {
        auto wait_duration = std::chrono::duration<double>(context_->params.reasonable_timeout);
        auto status = future_->wait_for(wait_duration);

        if (status == std::future_status::ready) {
          try {
            result = future_->get(); // consume future
            future_ = std::nullopt;
            return result;
          }
          catch (const std::exception& e) {
            RCLCPP_ERROR(context_->logger, "Service call failed: %s", e.what());
            return nullptr;
          }
        }
      }

      // Wait for any switch controllers future to complete
      while (running_.load() && future_.has_value() && future_->valid()) {
        // TODO: Parameterize period
        auto status = future_->wait_for(std::chrono::milliseconds(100));

        if (status != std::future_status::ready)
          continue;

        try {
          result = future_->get(); // consume future
          future_ = std::nullopt;
          return result;
        }
        catch (const std::exception& e) {
          RCLCPP_ERROR(context_->logger, "Service call failed: %s", e.what());
          return nullptr;
        }
      }

      return result;
    }

    /**
     * Waits until the list_future_ is ready.
     * Polls running_ every 100ms to try stop
     */
    controller_manager_msgs::srv::ListControllers::Response::SharedPtr wait_for_list_future() {
      controller_manager_msgs::srv::ListControllers::Response::SharedPtr result = nullptr;

      if (!(running_.load() && list_future_.has_value() && list_future_->valid()))
        return nullptr;

      // Do initial patient waiting
      if (!context_->params.impatient) {
        auto wait_duration = std::chrono::duration<double>(context_->params.reasonable_timeout);
        auto status = list_future_->wait_for(wait_duration);

        if (status == std::future_status::ready) {
          try {
            result = list_future_->get();
            list_future_ = std::nullopt;
            return result;
          }
          catch (const std::exception& e) {
            RCLCPP_ERROR(context_->logger, "List service call failed: %s", e.what());
            list_future_ = std::nullopt;
            return nullptr;
          }
        }
      }

      // Wait for any switch controllers future to complete
      while (running_.load() && list_future_.has_value() && list_future_->valid()) {
        // TODO: Parameterize period
        auto status = list_future_->wait_for(std::chrono::milliseconds(100));

        if (status != std::future_status::ready)
          continue;

        try {
          result = list_future_->get();
          list_future_ = std::nullopt;
          return result;
        }
        catch (const std::exception& e) {
          RCLCPP_ERROR(context_->logger, "List service call failed: %s", e.what());
          list_future_ = std::nullopt;
          return nullptr;
        }
      }

      return result;
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
        std::lock_guard lock(context_->current_active_controllers_mutex);
        current_active_controllers = context_->current_active_controllers;
      }

      { // Copy active controls while holding a lock
        std::lock_guard lock(context_->desired_active_controllers_mutex);
        desired_active_controllers = context_->desired_active_controllers;
      }

      set_differences(current_active_controllers, desired_active_controllers, deactivate_set, activate_set);

      if (context_->params.log_controllers) {
        RCLCPP_INFO(context_->logger, "desired_active_controllers:");
        for (const auto id : desired_active_controllers)
          RCLCPP_INFO(context_->logger, "    - \"%s\" (id: %lu)", context_->ordering[id].c_str(), id);
        RCLCPP_INFO(context_->logger, "current_active_controllers:");
        for (const auto id : current_active_controllers)
          RCLCPP_INFO(context_->logger, "    - \"%s\" (id: %lu)", context_->ordering[id].c_str(), id);
        RCLCPP_INFO(context_->logger, " ");

        RCLCPP_INFO(context_->logger, "deactivate_set:");
        for (const auto id : deactivate_set)
          RCLCPP_INFO(context_->logger, "    - \"%s\" (id: %lu)", context_->ordering[id].c_str(), id);
        RCLCPP_INFO(context_->logger, "activate_set:");
        for (const auto id : activate_set)
          RCLCPP_INFO(context_->logger, "    - \"%s\" (id: %lu)", context_->ordering[id].c_str(), id);
        RCLCPP_INFO(context_->logger, " ");
      }

      // Convert the id sets into controller names
      std::vector<std::string> deactivate_names{};
      std::vector<std::string> activate_names{};

      deactivate_names.reserve(deactivate_set.size());
      activate_names.reserve(activate_set.size());

      // deactivate names are provided in reverse order
      for (auto it = deactivate_set.rbegin(); it != deactivate_set.rend(); ++it)
        deactivate_names.emplace_back(context_->ordering[*it]);

      for (auto id : activate_set)
        activate_names.emplace_back(context_->ordering[id]);

      if (!running())
        return;

      // Attempt the switch
      bool result = switch_controllers(deactivate_names, activate_names);

      // Repeatedly retry a fixed number of times, with a short delay between attempts
      const int max_attempts = context_->params.connection_retry_count;
      for (int i = 1; (max_attempts < 0 || i < max_attempts) && !result && running_.load(); ++i) {
        // Wait for a short amount of time, or until the thread should stop running.
        wait_for(std::chrono::duration<double>(1.0 / context_->params.connection_retry_rate));
        if (!running_.load())
          return;

        result = switch_controllers(deactivate_names, activate_names);
      }

      if (running() && !result) {
        RCLCPP_ERROR(context_->logger, "%s/switch_controller not available after %d attempts. Stopping attempts.",
                     context_->params.controller_manager.c_str(), max_attempts);
      }

      if (!running())
        return;

      auto response = wait_for_future();
      if (!response)
        return;

      service_future_result(response);
    }

    /**
     * Makes the call to the controller manager to try switch the active controllers
     * \returns True if the controller manager service was available and the service was called.
     */
    bool switch_controllers(
        const std::vector<std::string> & controllers_to_deactivate,
        const std::vector<std::string> & controllers_to_activate)
    {
      if (controllers_to_deactivate.empty() && controllers_to_activate.empty()) {
        return true;
      }

      if (!context_->switch_controller_client->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(context_->logger, *context_->node->get_clock(), 2000,
                             "%s/switch_controller service not currently available.",
                             context_->params.controller_manager.c_str());
        return false;
      }

      const auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
      request->deactivate_controllers = controllers_to_deactivate;
      request->activate_controllers = controllers_to_activate;
      request->strictness = context_->params.strictness;
      request->activate_asap = context_->params.activate_asap;
      request->timeout = context_->params.timeout;

      future_ = context_->switch_controller_client->async_send_request(request);
      return true;
    }

    /**
     * Called when we get the result back from a switch controller request
     */
    void service_future_result(const controller_manager_msgs::srv::SwitchController::Response::SharedPtr & response) {
      if (!response->ok) {
        RCLCPP_ERROR(context_->logger, "Failed to switch controllers.");
        return;
      }

      // We can now assume context_->current_active_controllers_ represents the actual state of the controllers
      {
        // TODO: Confirm we wont cause deadlock
        std::lock_guard lock2(context_->current_active_controllers_mutex);
        std::lock_guard lock1(context_->desired_active_controllers_mutex);

        context_->current_active_controllers = context_->desired_active_controllers;
      }

      if (context_->params.log_controllers)
        RCLCPP_INFO(context_->logger, "Successfully switched controllers.\n");
    }

    /**
     * Called when we get the result back from a list controllers request
     * \returns whether anything in current_active_controllers actually changed as a result of this method call.
     */
    bool service_list_future_result(const controller_manager_msgs::srv::ListControllers::Response::SharedPtr & response) {
      bool changed = false;

      if (context_->params.log_controllers)
        RCLCPP_INFO(context_->logger, "Actual controller states from ros2_control:");

      {
        std::lock_guard lock2(context_->current_active_controllers_mutex);

        for (const auto& controller : response->controller)
        {
          // Ignore controllers not defined in teleop
          if (!context_->ordering.contains(controller.name)) {
            if (context_->params.log_controllers)
              RCLCPP_INFO(context_->logger, "  - \"%s\" (not defined in teleop)\t: %s",
                          controller.name.c_str(), controller.state.c_str());
            continue;
          }

          const auto is_active = controller.state == "active";
          const auto is_inactive = controller.state == "inactive";

          if (!is_active && !is_inactive)
          {
            // The controller is in some other weird state. Complain to the user, and move on
            RCLCPP_ERROR(context_->logger, "Controller \"%s\" is in state \"%s\". Treating it as inactive...",
                         controller.name.c_str(), controller.state.c_str());
          }

          const auto id = context_->ordering[controller.name];

          if (context_->params.log_controllers)
            RCLCPP_INFO(context_->logger, "  - \"%s\" (id: %lu)\t: %s",
                        controller.name.c_str(), id, controller.state.c_str());

          if (is_active) {
            auto [_, element_changed] = context_->current_active_controllers.insert(id);
            changed = changed || element_changed;
          }
          else {
            auto no_elements_erased = context_->current_active_controllers.erase(id);
            if (no_elements_erased > 0)
              changed = true;
          }
        }
      }

      if (context_->params.log_controllers) {
        RCLCPP_INFO(context_->logger, "Retrieved actual active controllers:");
        for (const auto id : context_->current_active_controllers)
        {
          const auto name = context_->ordering[id];
          RCLCPP_INFO(context_->logger, "  - \"%s\" (id: %lu)", name.c_str(), id);
        }
      }

      return changed;
    }

    bool get_actual_current_controllers() {
      const int max_attempts = context_->params.connection_retry_count;
      for (int i = 0; (max_attempts < 0 || i < max_attempts) && !context_->switch_controller_client->service_is_ready() && running(); ++i) {
        RCLCPP_WARN_THROTTLE(context_->logger, *context_->node->get_clock(), 2000, "%s/list_controllers service not currently available.", context_->params.controller_manager.c_str());
        wait_for(std::chrono::duration<double>(1.0 / context_->params.connection_retry_rate));
      }

      if (!running())
        return false;

      if (!context_->switch_controller_client->service_is_ready()) {
        RCLCPP_ERROR(context_->logger, "%s/list_controllers not available after %d attempts. Stopping attempts.",
                     context_->params.controller_manager.c_str(), max_attempts);
        return false;
      }

      if (context_->params.log_controllers)
        RCLCPP_INFO(context_->logger, "Getting the actual controllers states from the controller manager...");

      const auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
      list_future_ = context_->list_controllers_client->async_send_request(request);

      auto response = wait_for_list_future();
      if (!response)
        return false;

      service_list_future_result(response);

      if (!running())
        return false;

      return true;
    }

    /**
     * Called whenever a switch request finished without succeeding.
     */
    void on_switch_failure() {
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
    /// Any previous result from a service call
    std::optional<rclcpp::Client<controller_manager_msgs::srv::ListControllers>::FutureAndRequestId> list_future_;
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
      deactivate_id_sets.emplace_back(std::ref(context_->controllers_for_ids[cm_id]));
    const auto deactivate_ids = ControllerOrdering::merge_id_sets(deactivate_id_sets);

    // Get activate ids
    std::vector<std::reference_wrapper<std::set<size_t>>> activate_id_sets{};
    activate_id_sets.reserve(activate_cm_ids.size());
    for (const auto cm_id : activate_cm_ids)
      activate_id_sets.emplace_back(std::ref(context_->controllers_for_ids[cm_id]));
    const auto activate_ids = ControllerOrdering::merge_id_sets(activate_id_sets);

    // Hold lock
    std::lock_guard lock(context_->desired_active_controllers_mutex);

    // Remove from desired ids
    for (const auto id : deactivate_ids)
      context_->desired_active_controllers.erase(id);

    // Add to desired ids
    for (const auto id : activate_ids)
      context_->desired_active_controllers.insert(id);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Context> context_ = std::make_shared<Context>(node_);
  std::shared_ptr<Worker> worker_ = nullptr;
};

}  // namespace teleop

#endif  // CONTROL_MODE_CONTROLLER_MANAGER_MANAGER_HPP
