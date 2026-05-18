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
// Created by Bailey Chessum on 6/29/25.
//

#ifndef TELEOP_MODULAR_SWITCHCONTROLMODECOMMAND_HPP
#define TELEOP_MODULAR_SWITCHCONTROLMODECOMMAND_HPP

#include "Command.hpp"
#include <string>
#include <vector>

namespace teleop
{

class SwitchControlModeCommand final : public Command
{
public:
  void on_initialize(
    const std::string & prefix,
    const ParameterInterface::SharedPtr & parameters, CommandDelegate & context) override;

  void execute(CommandDelegate & context, const rclcpp::Time & now) override;

protected:
  struct Params
  {
    /// Set of controller names to activate
    std::vector<std::string> activate{};
    /// Set of controller names to deactivate
    std::vector<std::string> deactivate{};

    // Deprecated.
    std::optional<std::string> to = std::nullopt;
  };

  Params params_{};
  rclcpp::Logger logger_ = rclcpp::get_logger("switch_control_mode");
};

}  // namespace teleop

#endif  // TELEOP_MODULAR_SWITCHCONTROLMODECOMMAND_HPP
