#include "teleop_modular/control_modes/ControlMode.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace teleop::control_mode
{

ControlMode::~ControlMode()
{
  // Check from ControllerInterfaceBase
  if (node_.get() && rclcpp::ok() &&
      node_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
  {
    node_->shutdown();
  }
}

return_type ControlMode::init(const std::string& name, const std::string& node_namespace,
                              const rclcpp::NodeOptions& node_options)
{
  name_ = name;
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(name_, node_namespace, node_options, false);

  // Perform child class initialization
  switch (on_init())
  {
    case CallbackReturn::SUCCESS:
      break;
    default:
      node_->shutdown();
      return return_type::ERROR;
  }

  node_->register_on_configure(std::bind(&ControlMode::on_configure, this, std::placeholders::_1));
  node_->register_on_activate(std::bind(&ControlMode::on_configure, this, std::placeholders::_1));
  node_->register_on_deactivate(std::bind(&ControlMode::on_deactivate, this, std::placeholders::_1));
  node_->register_on_error(std::bind(&ControlMode::on_error, this, std::placeholders::_1));
  node_->register_on_shutdown(std::bind(&ControlMode::on_shutdown, this, std::placeholders::_1));
  node_->register_on_cleanup(std::bind(&ControlMode::on_cleanup, this, std::placeholders::_1));

  return return_type::OK;
}

const rclcpp_lifecycle::State& ControlMode::get_lifecycle_state() const
{
  if (!node_.get())
  {
    RCLCPP_ERROR(rclcpp::get_logger(get_name()), "Lifecycle node accessed without being initialized!");
    throw std::runtime_error("Lifecycle node accessed without being initialized!");
  }

  return node_->get_current_state();
}

}  // namespace teleop::control_mode