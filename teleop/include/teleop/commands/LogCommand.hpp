//
// Created by nova on 6/29/25.
//

#ifndef TELEOP_LOGCOMMAND_HPP
#define TELEOP_LOGCOMMAND_HPP
#include "Command.hpp"

namespace teleop
{

class LogCommand final : public Command
{
public:
  void on_initialize(const std::string& prefix, const ParameterInterface::SharedPtr& parameters) override;

  void execute(CommandDelegate& context, const rclcpp::Time& now) override;

protected:
  struct Params
  {
    std::string message = "";
  };

  Params params_{};
};

}  // namespace teleop

#endif  // TELEOP_LOGCOMMAND_HPP
