//
// Created by nova on 7/6/25.
//

#include "teleop/input_sources/InputSourceHandle.hpp"
#include <functional>
#include <set>

#include "colors.hpp"

namespace teleop::internal
{
InputSourceHandle::InputSourceHandle(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& parameters,
                                     InputManager& inputs, const std::shared_ptr<InputSource>& source)
  : inputs_(std::ref(inputs)), parameters_(parameters), source_(source)
{
  declare_and_link_inputs();
}

InputSourceHandle::InputSourceHandle(InputManager& inputs, const std::shared_ptr<InputSource>& source)
  : inputs_(std::ref(inputs)), source_(source), parameters_(source->get_node()->get_node_parameters_interface())
{
  declare_and_link_inputs();
}

void InputSourceHandle::update(const rclcpp::Time& now) const
{
  source_->update(now);
}

void InputSourceHandle::add_definitions_to_inputs() const
{
  // Add declarations to inputs
  for (auto& button : button_definitions_)
    for (auto& definition : button.references)
      button.input->add_definition(definition);

  for (auto& axis : axis_definitions_)
    for (auto& definition : axis.references)
      axis.input->add_definition(definition);
}

void InputSourceHandle::declare_and_link_inputs()
{
  const auto logger = source_->get_node()->get_logger();
  // TODO: Check if inputs are already linked, and unlink them

  auto declarations = source_->export_inputs();
  const auto remap_params = get_remap_params();
  remap(declarations, remap_params);

  add_definitions_to_inputs();
}

InputSourceHandle::RemapParams InputSourceHandle::get_remap_params()
{
  RemapParams remap_params;
  auto& inputs = inputs_.get();

  for (const auto& button : inputs.get_buttons())
    if (auto params = get_remap_button_params(button->get_name()); params.has_value())
      remap_params.buttons.emplace_back(*params);

  for (const auto& axis : inputs.get_axes())
    if (auto params = get_remap_axis_params(axis->get_name()); params.has_value())
      remap_params.axes.emplace_back(*params);

  return remap_params;
}

std::optional<InputSourceHandle::RemapButtonParams> InputSourceHandle::get_remap_button_params(const std::string& name)
{
  const auto logger = source_->get_node()->get_logger();
  const auto prefix = "remap.buttons." + name + ".";

  // Declare from param
  rcl_interfaces::msg::ParameterDescriptor from_descriptor;
  from_descriptor.name = prefix + "from";
  from_descriptor.description = "The name of the button to remap values from.";
  parameters_->declare_parameter(from_descriptor.name, rclcpp::PARAMETER_STRING, from_descriptor);

  rclcpp::Parameter from_param;
  auto type_result = parameters_->get_parameter(from_descriptor.name, from_param);

  if (!type_result || from_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
  {
    RCLCPP_DEBUG(logger, "  %s not remapped", from_descriptor.name.c_str());
    return std::nullopt;
  }

  RCLCPP_DEBUG(logger, C_INPUT "  %s = %s" C_RESET, from_descriptor.name.c_str(), from_param.as_string().c_str());
  return RemapButtonParams{ name, from_param.as_string() };
}

std::optional<InputSourceHandle::RemapAxisParams> InputSourceHandle::get_remap_axis_params(const std::string& name)
{
  const auto logger = source_->get_node()->get_logger();
  const auto prefix = "remap.axes." + name + ".";

  // Declare from param
  rcl_interfaces::msg::ParameterDescriptor from_descriptor;
  from_descriptor.name = prefix + "from";
  from_descriptor.description = "The name of the axis to remap values from.";
  parameters_->declare_parameter(from_descriptor.name, rclcpp::PARAMETER_STRING, from_descriptor);

  rclcpp::Parameter from_param;
  auto type_result = parameters_->get_parameter(from_descriptor.name, from_param);

  if (!type_result || from_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
  {
    RCLCPP_DEBUG(logger, "  %s not remapped", from_descriptor.name.c_str());
    return std::nullopt;
  }

  RCLCPP_DEBUG(logger, C_INPUT "  %s.from = %s" C_RESET, from_descriptor.name.c_str(), from_param.as_string().c_str());
  return RemapAxisParams{ name, from_param.as_string() };
}

void InputSourceHandle::remap(InputSource::InputDeclarationSpans declarations, RemapParams remap_params)
{
  const auto logger = source_->get_node()->get_logger();
  auto& inputs = inputs_.get();

  // Stores, for each exported button, whether the button is remapped
  std::vector<bool> is_button_remapped(declarations.button_names.size(), false);

  // Reset definitions
  button_definitions_.clear();
  button_definitions_.reserve(declarations.button_names.size());

  // Create definitions for each remapped input, marking which ones have been remapped
  auto& buttons = inputs.get_buttons();
  for (auto& [name, from] : remap_params.buttons)
  {
    // Find the index for the 'from'
    const auto it = std::find(declarations.button_names.begin(), declarations.button_names.end(), from);
    const size_t index = std::distance(declarations.button_names.begin(), it);

    const auto reference = std::ref(declarations.buttons[index]);

    button_definitions_.emplace_back(buttons[name], std::vector{ reference });
    is_button_remapped[index] = true;
  }

  // Create definitions for unmapped inputs
  for (size_t i = 0; i < declarations.button_names.size(); ++i)
  {
    if (is_button_remapped[i])
      continue;

    const auto& name = declarations.button_names[i];
    const auto reference = std::ref(declarations.buttons[i]);

    button_definitions_.emplace_back(buttons[name], std::vector{ reference });
  }

  // Do the same thing again, but for axes
  // TODO: Do a template function or something to remove duplicate code

  // Stores, for each exported axis, whether the axis is remapped
  std::vector<bool> is_axis_remapped(declarations.axis_names.size(), false);

  // Reset definitions
  axis_definitions_.clear();
  axis_definitions_.reserve(declarations.axis_names.size());

  // Create definitions for each remapped input, marking which ones have been remapped
  auto& axes = inputs.get_axes();
  for (auto& [name, from] : remap_params.axes)
  {
    // Find the index for the 'from'
    const auto it = std::find(declarations.axis_names.begin(), declarations.axis_names.end(), from);
    const size_t index = std::distance(declarations.axis_names.begin(), it);

    const auto reference = std::ref(declarations.axes[index]);

    axis_definitions_.emplace_back(axes[name], std::vector{ reference });
    is_axis_remapped[index] = true;
  }

  // Create definitions for unmapped inputs
  for (size_t i = 0; i < declarations.axis_names.size(); ++i)
  {
    if (is_axis_remapped[i])
      continue;

    const auto& name = declarations.axis_names[i];
    const auto reference = std::ref(declarations.axes[i]);

    axis_definitions_.emplace_back(axes[name], std::vector{ reference });
  }
}
}  // namespace teleop::internal
