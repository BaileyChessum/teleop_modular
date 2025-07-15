//
// Created by Bailey Chessum on 10/6/25.
//

#include "teleop_modular/input_sources/InputSource.hpp"

namespace teleop::input_source
{

void InputSource::initialize(const std::shared_ptr<rclcpp::Node>& node, const std::string& name,
                             const std::weak_ptr<InputSourceUpdateDelegate>& delegate)
{
  node_ = node;
  name_ = name;
  delegate_ = delegate;

  on_initialize();
}

void InputSource::update(const rclcpp::Time& now)
{
  InputValueSpans spans{ span(button_values_), span(axis_values_) };

  on_update(now, spans);
}

bool InputSource::request_update(const rclcpp::Time& now) const
{
  const auto delegate = delegate_.lock();

  if (!delegate)
  {
    RCLCPP_FATAL(node_->get_logger(), "InputSource %s's delegate weak_ptr is invalid!", name_.c_str());
    return false;
  }

  const auto time_to_send = now.nanoseconds() == 0 ? node_->get_clock()->now() : now;
  delegate->on_input_source_requested_update(time_to_send);
  return true;
}

InputSource::InputDeclarationSpans InputSource::export_inputs()
{
  button_names_.clear();
  button_values_.clear();
  InputDeclarationList button_declarations(button_names_, button_values_);
  export_buttons(button_declarations);

  axis_names_.clear();
  axis_values_.clear();
  InputDeclarationList axis_declarations(axis_names_, axis_values_);
  export_axes(axis_declarations);

  return InputDeclarationSpans{ span(button_values_), span(axis_values_), span(button_names_), span(axis_names_) };
}

}  // namespace teleop::input_source
