//
// Created by nova on 6/29/25.
//

#ifndef TELEOP_MODULAR_SWITCHCONTROLMODECOMMAND_HPP
#define TELEOP_MODULAR_SWITCHCONTROLMODECOMMAND_HPP

#include "Command.hpp"

namespace teleop
{

class SwitchControlModeCommand final : public Command
{
public:
  void on_initialize(
    const std::string & prefix,
    const ParameterInterface::SharedPtr & parameters) override;

  void execute(CommandDelegate & context, const rclcpp::Time & now) override;

protected:
  struct Params
  {
    std::string to = "";
  };

  Params params_{};
};

}  // namespace teleop

#endif  // TELEOP_MODULAR_SWITCHCONTROLMODECOMMAND_HPP
