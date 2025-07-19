//
// Created by Bailey Chessum on 10/6/25.
//

#include "input_source/input_source.hpp"

namespace input_source
{

return_type InputSource::init(
  const std::shared_ptr<rclcpp::Node> & node, const std::string & name,
  const std::weak_ptr<UpdateDelegate> & delegate)
{
  node_ = node;
  name_ = name;
  delegate_ = delegate;

  return on_init();
}

return_type InputSource::update(const rclcpp::Time & now)
{
  InputValueSpans spans{span(button_values_), span(axis_values_)};

  return on_update(now, spans);
}

return_type InputSource::request_update(const rclcpp::Time & now) const
{
  const auto delegate = delegate_.lock();

  if (!delegate) {
    RCLCPP_FATAL(
      node_->get_logger(), "InputSource %s's delegate weak_ptr is invalid!",
      name_.c_str());
    return return_type::ERROR;
  }

  const auto time_to_send = now.nanoseconds() == 0 ? node_->get_clock()->now() : now;
  delegate->on_input_source_requested_update(time_to_send);
  return return_type::OK;
}

InputDeclarationSpans InputSource::export_inputs()
{
  button_names_.clear();
  button_values_.clear();
  InputDeclarationList button_declarations(button_names_, button_values_);
  export_buttons(button_declarations);

  axis_names_.clear();
  axis_values_.clear();
  InputDeclarationList axis_declarations(axis_names_, axis_values_);
  export_axes(axis_declarations);

  return InputDeclarationSpans{{span(button_values_), span(axis_values_)}, span(button_names_),
    span(axis_names_)};
}

}  // namespace input_source
