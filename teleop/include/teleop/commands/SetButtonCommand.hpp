//
// Created by nova on 7/4/25.
//

#ifndef TELEOP_SETBUTTONCOMMAND_HPP
#define TELEOP_SETBUTTONCOMMAND_HPP

#include "Command.hpp"
#include <string>
#include <rclcpp/time.hpp>
#include "CommandDelegate.hpp"

namespace teleop
{

class SetButtonCommand final : public Command
{
public:
  void on_initialize(const std::string& prefix, const ParameterInterface::SharedPtr& parameters) override;
  void execute(CommandDelegate& context, const rclcpp::Time& now) override;

protected:
  struct Params
  {
    std::string name;
    bool value = true;
  };

  Params params_{};
};

}  // namespace teleop

#endif  // TELEOP_SETBUTTONCOMMAND_HPP
