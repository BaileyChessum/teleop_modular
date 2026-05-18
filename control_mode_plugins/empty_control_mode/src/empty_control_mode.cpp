// Copyright 2026 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#include "empty_control_mode/empty_control_mode.hpp"

namespace empty_control_mode
{

EmptyControlMode::EmptyControlMode() = default;

EmptyControlMode::~EmptyControlMode() = default;

return_type EmptyControlMode::on_init()
{
  return return_type::OK;
}

void EmptyControlMode::on_configure_inputs(Inputs)
{
}

CallbackReturn EmptyControlMode::on_activate(const State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn EmptyControlMode::on_deactivate(const State &)
{
  return CallbackReturn::SUCCESS;
}

return_type EmptyControlMode::on_update(const rclcpp::Time &, const rclcpp::Duration &)
{
  return return_type::OK;
}

CallbackReturn EmptyControlMode::on_error(const State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn EmptyControlMode::on_cleanup(const State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn EmptyControlMode::on_shutdown(const State &)
{
  return CallbackReturn::SUCCESS;
}

}  // namespace empty_control_mode

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(empty_control_mode::EmptyControlMode, control_mode::ControlMode);
