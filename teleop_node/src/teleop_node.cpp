// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#include "teleop_node/teleop_node.hpp"

#include "teleop_core/colors.hpp"
#include "teleop_core/control_modes/ControlModeManager.hpp"
#include <iostream>
#include "teleop_core/utilities/get_parameter.hpp"

using namespace std::chrono_literals;

namespace teleop
{

TeleopNode::TeleopNode(const std::shared_ptr<rclcpp::Node> & node)
: node_(node), states_(), events_(inputs_)
{
  auto parameters = node->get_node_parameters_interface();

  // Create publishers
  params_.log_inputs = utils::get_parameter_or_default<bool>(
    parameters, "log_inputs", "When true, all inputs will be logged to help with configuration.",
    false);
  params_.update_rate = utils::get_parameter_or_default<double>(
    parameters, "update_rate",
    "The maximum rate at which updates should occur, and "
    "hence the max rate at which commands are sent. "
    "Leaving this unset makes the max update rate "
    "unlimited.",
    0.0);
}

void TeleopNode::initialize(const std::weak_ptr<rclcpp::Executor> & executor)
{
  const auto logger = get_node()->get_logger();
  RCLCPP_DEBUG(logger, "TeleopNode::init(): Creating inputs");

  RCLCPP_DEBUG(logger, "TeleopNode::init(): Creating control modes.");
  control_mode_manager_ = std::make_shared<internal::ControlModeManager>(get_node(), executor);
  control_mode_manager_->configure(inputs_);

  RCLCPP_DEBUG(logger, "TeleopNode::init(): Creating commands.");
  commands_ = std::make_shared<internal::CommandManager>(get_node(), shared_from_this());
  commands_->configure(events_.get_events());

  log_existing_inputs();

  // Input source initialization and setup depends on commands and control modes to first request the inputs that want
  // populated.
  RCLCPP_DEBUG(logger, "TeleopNode::init(): Creating input sources.");
  input_source_manager_ = std::make_shared<internal::InputSourceManager>(
    get_node(), executor,
    inputs_);
  input_source_manager_->configure(inputs_);

  InputManager::Props input_props{};
//  input_source_manager_->link_inputs(input_props);
  inputs_.init(input_props);

  RCLCPP_DEBUG(logger, "TeleopNode::init(): Starting...");
  control_mode_manager_->activate_initial_control_mode();

  const auto now = get_node()->now();
  events_.get_events()["start"]->invoke();
  events_.update(now);
  RCLCPP_DEBUG(logger, "TeleopNode::init(): Fully initialized!");
}

void TeleopNode::log_all_inputs()
{
  const auto logger = get_node()->get_logger();

/*
  for (const auto & axis : inputs_.get_axes()) {
    if (!axis) {
      continue;
    }
    RCLCPP_DEBUG(logger, C_INPUT "  %s\t%f", axis.get_name().c_str(), axis.value());

    if (axis->changed()) {
      RCLCPP_INFO(logger, C_INPUT "  %s\t%f", axis.get_name().c_str(), axis.value());
    }
  }
  for (const auto & button : inputs_.get_buttons()) {
    if (!button) {
      continue;
    }
    RCLCPP_DEBUG(logger, C_INPUT "  %s\t%d", button.get_name().c_str(), button.value());

    if (button->changed()) {
      RCLCPP_INFO(logger, C_INPUT "  %s\t%d", button.get_name().c_str(), button.value());
    }
  }
  for (auto & event : events_.get_events()) {
    if (!event) {
      continue;
    }

    if (event->is_invoked()) {
      RCLCPP_INFO(logger, C_QUIET "  %s invoked!", event->get_name().c_str());
    }
  }
  */
}

void TeleopNode::log_existing_inputs()
{
  std::stringstream log;

  // if (inputs_.get_axes().size() > 0) {
  //   log << C_INPUT "\tAxes:\n" C_RESET;
  //
  //   for (const auto & axis : inputs_.get_axes()) {
  //     log << C_INPUT "\t  " << axis->get_name() << "\t" << axis.value() << "\n" C_RESET;
  //   }
  // }
  //
  // if (inputs_.get_buttons().size() > 0) {
  //   log << C_INPUT "\tButtons:\n" C_RESET;
  //
  //   for (const auto & button : inputs_.get_buttons()) {
  //     log << C_INPUT "\t  " << button->get_name() << "\t" << button.value() << "\n" C_RESET;
  //   }
  // }

  RCLCPP_DEBUG(
    get_node()->get_logger(), C_TITLE "Registered inputs:\n" C_RESET "%s\n",
    log.str().c_str());
}

void TeleopNode::service_input_updates()
{
  const auto logger = get_node()->get_logger();
  RCLCPP_DEBUG(
    logger,
    "TeleopNode::service_input_updates(): Starting input update servicing loop.");

  rclcpp::Time previous = get_node()->now();
  rclcpp::Rate rate(params_.update_rate > 0 ? params_.update_rate : 1000);

  while (program_running_) {
    const auto now = input_source_manager_->wait_for_update();
    const auto period = now - previous;

    input_source_manager_->update(now);
    inputs_.update(now);
    events_.update(now);

    // Log inputs
    if (params_.log_inputs) {
      log_all_inputs();
    }

    control_mode_manager_->update(now, period);

    // TODO: enforce max update rate here
    previous = now;

    // Enforce max update rate
    if (params_.update_rate > 0 && program_running_) {
      rate.sleep();
    }
  }
}

std::shared_ptr<rclcpp::Node> TeleopNode::get_node() const
{
  return node_;
}

InputManager & TeleopNode::get_inputs()
{
  return inputs_;
}

state::StateManager & TeleopNode::get_states()
{
  return states_;
}

const std::shared_ptr<internal::ControlModeManager> TeleopNode::get_control_modes() const
{
  return control_mode_manager_;
}

void TeleopNode::stop()
{
  input_source_manager_->on_input_source_requested_update(node_->now());
  program_running_ = false;
  // TODO: Hold the thread object in the class so it can be joined here.
}

TeleopNode::~TeleopNode()
{
  control_mode_manager_.reset();
}

}  // namespace teleop

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::string node_name = "teleop_node";

  // Manual CLI parsing to get --node-name
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--node-name" && i + 1 < argc) {
      node_name = argv[i + 1];
      ++i;  // Skip next argv as it's consumed
    }
  }

  const auto node = std::make_shared<rclcpp::Node>(node_name);
  const auto teleop_modular = std::make_shared<teleop::TeleopNode>(node);
  const auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);

  std::cout << "\n";
  teleop_modular->initialize(executor);

  {
    std::thread main_update_thread(&teleop::TeleopNode::service_input_updates, teleop_modular);
    executor->spin();

    teleop_modular->stop();
    main_update_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
