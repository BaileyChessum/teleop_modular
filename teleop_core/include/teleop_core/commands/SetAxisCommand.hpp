// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Created by Bailey Chessum on 31/7/25.
//

#ifndef TELEOP_CORE_SETAXISCOMMAND_HPP
#define TELEOP_CORE_SETAXISCOMMAND_HPP

#include "teleop_core/commands/Command.hpp"

namespace teleop
{

class SetAxisCommand : public Command
{
public:
  SetAxisCommand() = default;
  void on_initialize(
    const std::string & prefix,
    const ParameterInterface::SharedPtr & parameters,
    CommandDelegate & context) override;

  void execute(CommandDelegate & context, const rclcpp::Time & now) override;

protected:
  struct Params
  {
    std::string name;
    float value = 0.0;
  };

  Params params_{};
};

}  // namespace teleop

#endif  // TELEOP_CORE_SETAXISCOMMAND_HPP
