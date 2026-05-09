// Copyright 2026 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#ifndef EMPTY_CONTROL_MODE__EMPTY_CONTROL_MODE_HPP_
#define EMPTY_CONTROL_MODE__EMPTY_CONTROL_MODE_HPP_

#include <rclcpp/time.hpp>
#include "control_mode/control_mode.hpp"
#include "empty_control_mode/visibility_control.h"

namespace empty_control_mode
{

using namespace control_mode;

class EMPTY_CONTROL_MODE_PUBLIC EmptyControlMode : public ControlMode
{
public:
  EmptyControlMode();

  return_type on_init() override;
  void on_configure_inputs(Inputs inputs) override;
  return_type on_update(const rclcpp::Time & now, const rclcpp::Duration & period) override;

  CallbackReturn on_activate(const State & previous_state) override;
  CallbackReturn on_deactivate(const State & previous_state) override;
  CallbackReturn on_cleanup(const State & previous_state) override;
  CallbackReturn on_error(const State & previous_state) override;
  CallbackReturn on_shutdown(const State & previous_state) override;

protected:
  ~EmptyControlMode() override;
};

}  // namespace empty_control_mode

#endif  // EMPTY_CONTROL_MODE__EMPTY_CONTROL_MODE_HPP_
