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
    if (thread_running_)
      stop_thread();
  }

  void register_controllers_for_ids(const std::vector<std::reference_wrapper<std::vector<std::string>>>& controller_names_for_ids)
  {
    if (thread_running_)
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


  void main_thread() {
    RCLCPP_DEBUG(logger_, "Starting main thread loop.");

    while (thread_running_) {
      wait_for_change();
      update_controllers();
    }

    RCLCPP_DEBUG(logger_, "Main thread loop ending.");
  }

  void update_controllers() {









  }

  void switch_active(const std::vector<size_t> & activate_cm_ids, const std::vector<size_t> & deactivate_cm_ids) {
    // Get deactivate ids
    std::vector<std::reference_wrapper<std::set<size_t>>> deactivate_id_sets{};
    deactivate_id_sets.reserve(deactivate_cm_ids.size());
    for (const auto cm_id : deactivate_cm_ids)
      deactivate_id_sets.emplace_back(std::ref(controllers_for_ids_[cm_id]));
    const auto deactivate_ids = ControllerOrdering::merge_id_sets(deactivate_id_sets);

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

private:

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
    if (thread_running_)
      stop_thread();

    thread_running_ = true;
    thread_ = std::thread(&teleop::ControllerManagerManager::main_thread, this);
  }

  std::vector<std::set<size_t>> controllers_for_ids_{};  //< set of controller ids for each control mode id
  ControllerOrdering ordering_{};

  std::set<size_t> current_active_controllers_{};
  std::set<size_t> desired_active_controllers_{};

  rclcpp::Node::SharedPtr node_;
  std::thread thread_;
  bool thread_running_ = false;

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
