//
// Created by nova on 7/6/25.
//

#include "teleop/input_sources/InputSourceHandle.hpp"
#include <functional>

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

void InputSourceHandle::declare_and_link_inputs()
{
  // TODO: Check if inputs are already linked, and unlink them

  auto declarations = source_->export_inputs();
  const auto remap_params = get_remap_params();
  const auto remapping = remap(declarations, remap_params);

  auto& inputs = inputs_.get();

  button_definitions_.clear();
  button_definitions_.reserve(remapping.buttons.remapped_names.size());
  for (size_t i = 0; i < remapping.buttons.remapped_names.size(); i++)
  {
    const auto& name = remapping.buttons.remapped_names[i];
    auto& reference = remapping.buttons.references[i];

    button_definitions_.emplace_back(inputs.get_buttons()[name], reference);
    inputs.get_buttons()[name]->add_definition(reference);
  }

  axis_definitions_.clear();
  axis_definitions_.reserve(remapping.axes.remapped_names.size());
  for (size_t i = 0; i < remapping.axes.remapped_names.size(); i++)
  {
    const auto& name = remapping.axes.remapped_names[i];
    auto& reference = remapping.axes.references[i];

    axis_definitions_.emplace_back(inputs.get_axes()[name], reference);
    inputs.get_axes()[name]->add_definition(reference);
  }
}

InputSourceHandle::RemapParams InputSourceHandle::get_remap_params()
{
  RemapParams remap_params;
  auto& inputs = inputs_.get();

  for (auto& button : inputs.get_buttons())
    if (auto params = get_remap_button_params(button->get_name()))
      remap_params.buttons.emplace_back(*params);

  for (auto& axis : inputs.get_axes())
    if (auto params = get_remap_axis_params(axis->get_name()))
      remap_params.axes.emplace_back(*params);

  return remap_params;
}

std::optional<InputSourceHandle::RemapButtonParams> InputSourceHandle::get_remap_button_params(const std::string& name)
{
  const auto prefix = "remap.buttons." + name + ".";

  // Declare from param
  rcl_interfaces::msg::ParameterDescriptor from_descriptor;
  from_descriptor.name = prefix + "from";
  from_descriptor.description = "The name of the button to remap values from.";
  parameters_->declare_parameter(from_descriptor.name, rclcpp::PARAMETER_STRING, from_descriptor);

  rclcpp::Parameter from_param;
  auto type_result = parameters_->get_parameter(from_descriptor.name, from_param);

  if (!type_result || from_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    return std::nullopt;

  return RemapButtonParams{ from_param.as_string() };
}

std::optional<InputSourceHandle::RemapAxisParams> InputSourceHandle::get_remap_axis_params(const std::string& name)
{
  const auto prefix = "remap.axes." + name + ".";

  // Declare from param
  rcl_interfaces::msg::ParameterDescriptor from_descriptor;
  from_descriptor.name = prefix + "from";
  from_descriptor.description = "The name of the axis to remap values from.";
  parameters_->declare_parameter(from_descriptor.name, rclcpp::PARAMETER_STRING, from_descriptor);

  rclcpp::Parameter from_param;
  auto type_result = parameters_->get_parameter(from_descriptor.name, from_param);

  if (!type_result || from_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    return std::nullopt;

  return RemapAxisParams{ from_param.as_string() };
}

InputSourceHandle::Remapping InputSourceHandle::remap(InputSource::InputDeclarationSpans declarations,
                                                      InputSourceHandle::RemapParams remap_params)
{
  const auto logger = source_->get_node()->get_logger();
  Remapping remapping{};

  RCLCPP_DEBUG(logger, "Remapping inputs for %s", source_->get_name().c_str());

  std::map<std::string, std::string> button_name_remap;
  for (auto& button : remap_params.buttons)
  {
    button_name_remap[button.from] = button.name;
  }

  std::map<std::string, std::string> axis_name_remap;
  for (auto& axis : remap_params.axes)
  {
    axis_name_remap[axis.from] = axis.name;
  }

  remapping.buttons.references.reserve(declarations.button_names.size());
  remapping.buttons.remapped_names.reserve(declarations.button_names.size());
  for (size_t i = 0; i < declarations.button_names.size(); i++)
  {
    const auto& button_name = declarations.button_names[i];
    auto& button_value = declarations.buttons[i];

    // Determine remapped name
    if (auto it = button_name_remap.find(button_name); it != button_name_remap.end())
    {
      RCLCPP_DEBUG(logger, "  Remapping button %s to %s", button_name.c_str(), it->second.c_str());
      remapping.buttons.remapped_names.emplace_back(it->second);
    }
    else
    {
      remapping.buttons.remapped_names.emplace_back(button_name);
    }

    // Determined remapped value reference
    remapping.buttons.references.emplace_back(std::ref(button_value));
  }

  remapping.axes.references.reserve(declarations.axis_names.size());
  remapping.axes.remapped_names.reserve(declarations.axis_names.size());
  for (size_t i = 0; i < declarations.axis_names.size(); i++)
  {
    const auto& axis_name = declarations.axis_names[i];
    auto& axis_value = declarations.axes[i];

    // Determine remapped name
    if (auto it = axis_name_remap.find(axis_name); it != axis_name_remap.end())
    {
      RCLCPP_DEBUG(logger, "  Remapping axis %s to %s", axis_name.c_str(), it->second.c_str());
      remapping.axes.remapped_names.emplace_back(it->second);
    }
    else
    {
      remapping.axes.remapped_names.emplace_back(axis_name);
    }

    // Determined remapped value reference
    remapping.axes.references.emplace_back(std::ref(axis_value));
  }

  return remapping;
}


}  // namespace teleop::internal