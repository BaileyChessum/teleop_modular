//
// Created by nova on 6/28/25.
//

#include "../../include/teleop/commands/Command.hpp"

namespace teleop {

void Command::initialize(
  const CommandDelegate::WeakPtr& context,
  const std::string& name,
  const std::vector<Event::SharedPtr>& on,
  const LoggingInterface::SharedPtr& logging,
  const ParameterInterface::SharedPtr& parameters) {

  context_ = context;
  name_ = name;
  on_ = on;
  logger_ = logging->get_logger();

  // Do command implementation specific parameterization
  on_initialize("commands." + name + ".", parameters);
}

void Command::on_event_invoked(const rclcpp::Time& now) {
  if (const auto context = context_.lock())
    execute(*context, now);
}

} // teleop