#include "teleop/teleop.hpp"

#include "colors.hpp"
#include "teleop/control_modes/ControlModeManager.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

namespace teleop
{

Teleop::Teleop(const std::shared_ptr<rclcpp::Node>& node) : node_(node), states_(inputs_)
{
  // Create publishers
  param_listener_ = std::make_shared<ParamListener>(Teleop::get_node());
  params_ = param_listener_->get_params();
}

void Teleop::initialize(const std::weak_ptr<rclcpp::Executor>& executor)
{
  const auto logger = get_node()->get_logger();
  RCLCPP_DEBUG(logger, "Teleop::initialize(): Creating inputs");

  RCLCPP_DEBUG(logger, "Teleop::initialize(): Creating control modes.");
  control_mode_manager_ = std::make_shared<ControlModeManager>(get_node(), executor);
  control_mode_manager_->configure(inputs_);

  log_existing_inputs();

  RCLCPP_DEBUG(logger, "Teleop::initialize(): Creating input sources.");
  input_source_manager_ = std::make_shared<InputSourceManager>(get_node(), executor, inputs_);
  input_source_manager_->configure(param_listener_, inputs_);

  RCLCPP_DEBUG(logger, "Teleop::initialize(): Creating commands.");
  commands_ = std::make_shared<CommandManager>(get_node(), shared_from_this());
  commands_->configure(inputs_);

  RCLCPP_DEBUG(logger, "Teleop::initialize(): Fully initialized!");
}

void Teleop::log_all_inputs()
{
  const auto logger = get_node()->get_logger();

  for (const auto& axis : inputs_.get_axes())
  {
    if (!axis)
      continue;
    RCLCPP_DEBUG(logger, C_INPUT "  %s\t%f", axis->get_name().c_str(), axis->value());

    if (axis->changed())
    {
      RCLCPP_INFO(logger, C_INPUT "  %s\t%f", axis->get_name().c_str(), axis->value());
    }
  }
  for (const auto& button : inputs_.get_buttons())
  {
    if (!button)
      continue;
    RCLCPP_DEBUG(logger, C_INPUT "  %s\t%d", button->get_name().c_str(), button->value());

    if (button->changed())
    {
      RCLCPP_INFO(logger, C_INPUT "  %s\t%d", button->get_name().c_str(), button->value());
    }
  }
  for (auto& event : inputs_.get_events())
  {
    if (!event)
      continue;

    if (event->is_invoked())
    {
      RCLCPP_INFO(logger, C_QUIET "  %s invoked!", event->get_name().c_str());
    }
  }
}

void Teleop::log_existing_inputs()
{
  std::stringstream log;

  if (inputs_.get_axes().size() > 0) {
    log << C_INPUT "\tAxes:\n" C_RESET;

    for (const auto& axis : inputs_.get_axes()) {
      log << C_INPUT "\t  " << axis->get_name() << "\t" << axis->value() << "\n" C_RESET;
    }
  }

  if (inputs_.get_buttons().size() > 0) {
    log << C_INPUT "\tButtons:\n" C_RESET;

    for (const auto& button : inputs_.get_buttons()) {
      log << C_INPUT "\t  " << button->get_name() << "\t" << button->value() << "\n" C_RESET;
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), C_TITLE "Registered inputs:\n" C_RESET "%s", log.str().c_str());
}

void Teleop::service_input_updates()
{
  const auto logger = get_node()->get_logger();
  RCLCPP_DEBUG(logger, "Teleop::service_input_updates(): Starting input update servicing loop.");

  rclcpp::Time previous = get_node()->now();
  rclcpp::Rate rate(params_.update_rate > 0 ? params_.update_rate : 1000);

  while (program_running_)
  {
    const auto now = input_source_manager_->wait_for_update();
    const auto period = now - previous;

    input_source_manager_->update(now);
    inputs_.update(now);

    // Log inputs
    if (params_.log_inputs)
    {
      log_all_inputs();
    }

    control_mode_manager_->update(now, period);

    // TODO: enforce max update rate here
    previous = now;

    // Enforce max update rate
    if (params_.update_rate > 0)
    {
      rate.sleep();
    }
  }
}

std::shared_ptr<rclcpp::Node> Teleop::get_node() const
{
  return node_;
}

const InputManager& Teleop::get_inputs() const
{
  return inputs_;
}

StateManager& Teleop::get_states()
{
  return states_;
}

const std::shared_ptr<ControlModeManager> Teleop::get_control_modes() const
{
  return control_mode_manager_;
}

void Teleop::stop()
{
  input_source_manager_->on_input_source_requested_update(node_->now());
  program_running_ = false;
  // TODO: Hold the thread object in the class so it can be joined here.
}

Teleop::~Teleop()
{
  control_mode_manager_.reset();
}

}  // namespace teleop

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("teleop");
  const auto teleop = std::make_shared<teleop::Teleop>(node);
  const auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);

  std::cout << "\n";
  teleop->initialize(executor);

  {
    std::thread main_update_thread(&teleop::Teleop::service_input_updates, teleop);
    executor->spin();

    teleop->stop();
    main_update_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
