//
// Created by nova on 13/09/2025.
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

namespace teleop
{

/**
 * Helper class for managing the ros2_control controller manager
 */
class ControllerManagerManager
{
public:
  ControllerManagerManager(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)), logger_(node_->get_logger().get_child("controller_manager_manager"))
  {

  }

  ~ControllerManagerManager() {
    if (thread_running_.load())
      stop_thread();
  }

  void register_controllers_for_ids(const std::vector<std::reference_wrapper<std::vector<std::string>>>& controller_names_for_ids)
  {
    if (thread_running_.load())
      stop_thread();

    for (const auto& names_ref : controller_names_for_ids)
      ordering_.add(names_ref.get());
    ordering_.sort();

    controllers_for_ids_.clear();
    for (const auto& names_ref : controller_names_for_ids)
    {
      std::set<size_t> ids{};
      ordering_.names_to_ids(names_ref.get(), ids);

      controllers_for_ids_.emplace_back(std::move(ids));
    }

    start_thread();
  }

  /**
   * Function run by thread_
   */
  void main_thread() {
    RCLCPP_DEBUG(logger_, "Starting main thread loop.");

    while (thread_running_.load()) {
      wait_for_change();

      if (!thread_running_.load())
        continue;

      update_controllers();
    }

    RCLCPP_DEBUG(logger_, "Main thread loop ending.");
  }

  /**
   * Tries to update the active controllers, given control mode ids to be activated and deactivated.
   */
  void switch_active(const std::vector<size_t> & activate_cm_ids, const std::vector<size_t> & deactivate_cm_ids) {
    mutate_desired_active_controllers(activate_cm_ids, deactivate_cm_ids);
    invoke_update();
  }

  void invoke_update() {
    {
      std::lock_guard lock(mutex_);
      should_update_ = true;
    }

    update_condition_.notify_one();
  }

private:

  void update_controllers() {
    std::set<size_t> desired_active_controllers;

    { // Copy active controls while holding a lock
      std::unique_lock lock(desired_active_controllers_mutex_);
      desired_active_controllers = desired_active_controllers_;
    }

    std::set<size_t> deactivate_set{};
    std::set<size_t> activate_set{};

    set_differences(current_active_controllers_, desired_active_controllers, deactivate_set, activate_set);

    // Convert the id sets into controller names
    std::vector<std::string> deactivate_names{};
    std::vector<std::string> activate_names{};

    deactivate_names.reserve(deactivate_set.size());
    activate_names.reserve(activate_set.size());

    // deactivate names are provided in reverse order
    for (auto it = deactivate_set.rbegin(); it != deactivate_set.rend(); ++it)
      deactivate_names.emplace_back(ordering_[*it]);

    for (auto id : activate_set)
      activate_names.emplace_back(ordering_[id]);

    // TODO: Send message to the controller manager

  }

  /**
   * Gets the differences of two sets left and right
   * \param[out] left_difference left - right
   * \param[out] right_difference right - left
   */
  static void set_differences(
      const std::set<size_t>& left,
      const std::set<size_t>& right,
      std::set<size_t>& left_difference,
      std::set<size_t>& right_difference)
  {
    left_difference.clear();
    right_difference.clear();

    auto l_it = left.begin();
    auto r_it = right.begin();

    while (l_it != left.end() && r_it != right.end()) {
      if (*l_it < *r_it) {
        left_difference.insert(*l_it);
        ++l_it;
      } else if (*l_it < *r_it) {
        right_difference.insert(*r_it);
        ++r_it;
      } else {
        // element is common to both sets, skip both
        ++l_it;
        ++r_it;
      }
    }

    // Finish inserting elements for the other set when we finish inserting elements for one set
    left_difference.insert(l_it, left.end());
    right_difference.insert(r_it, right.end());
  }

  void mutate_desired_active_controllers(const std::vector<size_t> & activate_cm_ids, const std::vector<size_t> & deactivate_cm_ids) {
    // Get deactivate ids
    std::vector<std::reference_wrapper<std::set<size_t>>> deactivate_id_sets{};
    deactivate_id_sets.reserve(deactivate_cm_ids.size());
    for (const auto cm_id : deactivate_cm_ids)
      deactivate_id_sets.emplace_back(std::ref(controllers_for_ids_[cm_id]));
    const auto deactivate_ids = ControllerOrdering::merge_id_sets(deactivate_id_sets);

    // Hold lock
    std::unique_lock lock(desired_active_controllers_mutex_);

    // Remove from desired ids
    for (const auto id : deactivate_ids)
      desired_active_controllers_.erase(id);

    // Get activate ids
    std::vector<std::reference_wrapper<std::set<size_t>>> activate_id_sets{};
    activate_id_sets.reserve(activate_cm_ids.size());
    for (const auto cm_id : activate_cm_ids)
      activate_id_sets.emplace_back(std::ref(controllers_for_ids_[cm_id]));
    const auto activate_ids = ControllerOrdering::merge_id_sets(activate_id_sets);

    // Add to desired ids
    for (const auto id : activate_ids)
      desired_active_controllers_.insert(id);
  }

  /**
   * Blocks until desired_active_controllers_ changes
   */
  void wait_for_change() {
    std::unique_lock lock(mutex_);

    // Wait for changes to desired_active_controllers_
    update_condition_.wait(
        lock, [&] {
          return should_update_.load();
        });
  }

  void stop_thread() {
    thread_running_ = false;
    if (thread_.joinable())
      thread_.join();
  }

  void start_thread() {
    if (thread_running_.load())
      stop_thread();

    thread_running_ = true;
    thread_ = std::thread(&teleop::ControllerManagerManager::main_thread, this);
  }

  std::vector<std::set<size_t>> controllers_for_ids_{};  //< set of controller ids for each control mode id
  ControllerOrdering ordering_{};

  std::set<size_t> current_active_controllers_{};
  std::set<size_t> desired_active_controllers_{};
  /// Mutex to guard desired_active_controllers_ changes
  std::mutex desired_active_controllers_mutex_{};

  rclcpp::Node::SharedPtr node_;
  std::thread thread_;
  std::atomic<bool> thread_running_ = false;

  rclcpp::Logger logger_;

  // Concurrency control
  /// Mutex to guard update condition changes
  std::mutex mutex_;
  /// Lock that waits until should_update_ is true.
  std::condition_variable update_condition_;
  std::atomic<bool> should_update_ = false;


};

}  // namespace teleop

#endif  // CONTROL_MODE_CONTROLLER_MANAGER_MANAGER_HPP
