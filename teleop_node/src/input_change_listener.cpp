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
// Created by Bailey Chessum on 01/09/2025.
//

#include <teleop_core/colors.hpp>
#include "teleop_node/input_change_listener.hpp"
#include <cmath>
#include <utility>

namespace teleop
{

InputChangeListener::InputChangeListener(rclcpp::Logger logger, EventCollection & events, int significant_figures)
  : logger_(std::move(logger)), significant_figures_(significant_figures), events_(&events)
{
  sig_figures_mult_ = 1;
  while (significant_figures--)
    sig_figures_mult_ *= 10;

  epsilon_ = 0.5f / sig_figures_mult_;
}

void InputChangeListener::on_inputs_available(InputManager::Hardened& inputs)
{
  std::stringstream log;

  button_names_.clear();
  axis_names_.clear();
  previous_buttons_.clear();
  previous_axes_.clear();
  axes_.clear();
  buttons_.clear();

  button_names_.reserve(inputs.buttons.size());
  buttons_.reserve(inputs.buttons.size());
  previous_buttons_.reserve(inputs.buttons.size());
  size_t max_button_name_size = 8;
  log << C_RESET << "Buttons:\n" << C_RESET;
  RCLCPP_DEBUG(logger_, C_TITLE "Registered Buttons:" C_RESET);

  for (auto& [name, button] : inputs.buttons) {
    button_names_.emplace_back(name);
    buttons_.emplace_back(button);
    previous_buttons_.emplace_back(*button);

    if (name.size() > max_button_name_size)
      max_button_name_size = name.size();

    log << "  - " << C_INPUT;
    log.write(name.data(), static_cast<std::streamsize>(name.size()));
    log << C_RESET << "\n";
    RCLCPP_DEBUG(logger_, C_INPUT "\t%s" C_RESET, name.c_str());
  }

  axis_names_.reserve(inputs.axes.size());
  axes_.reserve(inputs.axes.size());
  previous_axes_.reserve(inputs.axes.size());
  size_t max_axis_name_size = 8;
  log << C_RESET << "Axes:\n" << C_RESET;
  RCLCPP_DEBUG(logger_, C_TITLE "Registered Axes:" C_RESET);

  for (auto& [name, axis] : inputs.axes) {
    axis_names_.emplace_back(name);
    axes_.emplace_back(axis);
    previous_axes_.emplace_back(*axis);

    if (name.size() > max_axis_name_size)
      max_axis_name_size = name.size();

    log << "  - " << C_INPUT;
    log.write(name.data(), static_cast<std::streamsize>(name.size()));
    log << C_RESET << "\n";
    RCLCPP_DEBUG(logger_, C_INPUT "%s" C_RESET, name.c_str());
  }

  std::string log_str = log.str();
  RCLCPP_INFO(
      logger_, C_TITLE "Registered inputs:" C_RESET "\n%.*s",
      static_cast<int>(log_str.size()), log_str.data());

  // Prevent axes and buttons from barely misaligning
  max_button_name_size_ = static_cast<int>(max_button_name_size);
  max_axis_name_size_ = static_cast<int>(max_axis_name_size);
  if (std::abs(max_axis_name_size_ - max_button_name_size_) <= 1) {
    const auto common_size = std::max(max_button_name_size_, max_axis_name_size_);
    max_button_name_size_ = max_axis_name_size_ = common_size;
  }
}

void InputChangeListener::update()
{
  for (size_t i = 0; i < axes_.size(); i++) {
    auto axis = axes_[i];

    if (!axis)
      continue;

    auto& previous_value = previous_axes_[i];
    auto new_value = std::round(*axis * sig_figures_mult_) / sig_figures_mult_;

    // Log if the value has changed enough
    if (std::abs(new_value - previous_value) >= epsilon_) {
      RCLCPP_INFO(logger_, C_INPUT "%-*s " C_QUIET "|" C_INPUT " % .*f" C_RESET,
                  max_axis_name_size_, axis_names_[i].c_str(), significant_figures_, new_value);
      previous_value = new_value;
    }
  }

  for (size_t i = 0; i < buttons_.size(); i++) {
    auto button = buttons_[i];

    if (!button) {
      continue;
    }

    auto& previous_value = previous_buttons_[i];

    if (previous_value != *button) {
      RCLCPP_INFO(logger_, C_INPUT "%-*s " C_QUIET "|" C_INPUT " %d" C_RESET,
                  max_button_name_size_, button_names_[i].c_str(), *button);
      previous_value = *button;
    }
  }

  if (!events_)
    return;

  for (auto & event : *events_) {
    if (!event)
      continue;

    if (event->is_invoked()) {
      RCLCPP_INFO(logger_, C_QUIET "%-8s\tinvoked", event->get_name().c_str());
    }
  }
}


}  // namespace teleop