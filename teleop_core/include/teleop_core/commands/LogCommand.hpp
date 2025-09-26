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
// Created by nova on 6/29/25.
//

#ifndef TELEOP_MODULAR_LOGCOMMAND_HPP
#define TELEOP_MODULAR_LOGCOMMAND_HPP
#include "Command.hpp"

namespace teleop
{

class LogCommand final : public Command
{
public:
  LogCommand() = default;
  void on_initialize(
    const std::string & prefix,
    const ParameterInterface::SharedPtr & parameters,
    CommandDelegate & context) override;

  void execute(CommandDelegate & context, const rclcpp::Time & now) override;

protected:
  struct Params
  {
    std::string message = "";
  };

  Params params_{};
};

}  // namespace teleop

#endif  // TELEOP_MODULAR_LOGCOMMAND_HPP
