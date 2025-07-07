//
// Created by nova on 7/6/25.
//

#include "teleop/input_sources/InputSourceHandle.hpp"
#include <functional>
#include <set>
#include "teleop/utilities/get_parameter.hpp"
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

template <>
inline void
InputSourceHandle::TransformedRemapValue<uint8_t, InputSourceHandle::TransformedRemapButtonFromAxis,
                                         InputSourceHandle::ButtonTransformParams>::update(const rclcpp::Time& now)
{
  value = 0;

  if (from.has_value())
    value = from.value().get();

  // Apply button from axis
  if (from_other.has_value())
  {
    auto& from_axis = from_other.value();

    if (from_axis.axis.get() < from_axis.threshold)
      value = 1;
  }

  if (!transform.has_value())
    return;

  // Apply transformations
  if (transform.value().invert)
    value = !value;
}

template <>
inline void
InputSourceHandle::TransformedRemapValue<float, InputSourceHandle::TransformedRemapAxisFromButtons,
                                         InputSourceHandle::AxisTransformParams>::update(const rclcpp::Time& now)
{
  value = 0.0f;

  if (from.has_value())
    value = from.value().get();

  // Apply axis from buttons
  if (from_other.has_value())
  {
    auto& from_buttons = from_other.value();

    if (from_buttons.negative.has_value() && from_buttons.negative.value().get())
      value -= 1.0f;

    if (from_buttons.positive.has_value() && from_buttons.positive.value().get())
      value += 1.0f;
  }

  if (!transform.has_value())
    return;

  // Apply transformations
  if (transform.value().invert)
    value = -value;
}

void InputSourceHandle::update(const rclcpp::Time& now)
{
  source_->update(now);

  for (auto& button : transformed_buttons_)
    button.update(now);

  for (auto& axis : transformed_axes_)
    axis.update(now);
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

  // Declare 'from' params first to check if we can exit early
  const auto from =
      get_parameter<std::string>(parameters_, prefix + "from", "The name of the button to remap values from.");

  const auto from_axis_name = get_parameter<std::string>(parameters_, prefix + "from_axis.name",
                                                         "The name of the axis to read the button value from.");

  // Exit early if nothing is defined
  if (!from.has_value() && !from_axis_name.has_value())
    return std::nullopt;

  std::optional<ButtonFromAxisParams> from_axis_params = std::nullopt;
  if (from_axis_name.has_value())
  {
    const auto threshold = get_parameter_or_default<float>(
        parameters_, prefix + "from_axis.threshold", "The threshold below which the button will be triggered.", 0.0);
    from_axis_params = ButtonFromAxisParams{ from_axis_name.value(), threshold };
  }

  const auto transform_params = get_button_transform_params(name);

  return RemapButtonParams{ name, from.value(), transform_params, from_axis_params };
}

std::optional<InputSourceHandle::ButtonTransformParams>
InputSourceHandle::get_button_transform_params(const std::string& name)
{
  const auto logger = source_->get_node()->get_logger();
  const auto prefix = "remap.buttons." + name + ".";

  InputSourceHandle::ButtonTransformParams params;

  params.invert = get_parameter_or_default<bool>(parameters_, prefix + "invert", "Invert the button value.", false);

  // When nullopt is returned, memory will be mapped directly. Otherwise, an intermediate value in memory will be set up
  if (params.invert)
    return params;
  else
    return std::nullopt;
}

std::optional<InputSourceHandle::RemapAxisParams> InputSourceHandle::get_remap_axis_params(const std::string& name)
{
  const auto logger = source_->get_node()->get_logger();
  const auto prefix = "remap.axes." + name + ".";

  // Declare 'from' params first to check if we can exit early
  const auto from =
      get_parameter<std::string>(parameters_, prefix + "from", "The name of the button to remap values from.");

  const auto from_button_negative = get_parameter<std::string>(parameters_, prefix + "from_buttons.negative",
                                                               "A button that when pressed will cause the axis to be "
                                                               "negative.");
  const auto from_button_positive = get_parameter<std::string>(parameters_, prefix + "from_buttons.positive",
                                                               "A button that when pressed will cause the axis to be "
                                                               "positive.");

  // Exit early if nothing is defined
  if (!from.has_value() && !from_button_negative.has_value() && !from_button_positive.has_value())
    return std::nullopt;

  std::optional<AxisFromButtonsParams> from_buttons_params = std::nullopt;
  if (from_button_negative.has_value() || from_button_positive.has_value())
  {
    from_buttons_params = AxisFromButtonsParams{ from_button_positive, from_button_negative };
  }

  const auto transform_params = get_axis_transform_params(name);

  return RemapAxisParams{ name, from.value(), transform_params, from_buttons_params };
}

std::optional<InputSourceHandle::AxisTransformParams>
InputSourceHandle::get_axis_transform_params(const std::string& name)
{
  const auto logger = source_->get_node()->get_logger();
  const auto prefix = "remap.axes." + name + ".";

  InputSourceHandle::AxisTransformParams params;

  params.invert = get_parameter_or_default<bool>(parameters_, prefix + "invert", "Invert the axis value.", false);

  auto range_out = get_parameter<std::vector<double>>(parameters_, prefix + "range.out",
                                                      "What range.in should be linearly mapped to.");
  auto range_clamp = get_parameter_or_default<bool>(parameters_, prefix + "range.clamp",
                                                    "Whether to clamp values to the specified range.", false);

  std::optional<AxisTransformParams::Range> range = std::nullopt;
  if (range_clamp || range_out.has_value())
  {
    auto range_in =
        get_parameter<std::vector<double>>(parameters_, prefix + "range.in", "The original range of input values.");

    // Enforce range_in must have 2 elements
    if (range_in.has_value() && range_in.value().size() != 2)
    {
      RCLCPP_ERROR(logger,
                   "Parameter %s.range.in should have exactly two elements, representing the upper and lower bound "
                   "respectively.",
                   prefix.c_str());
      range_in = std::nullopt;
    }

    // Enforce range_in must have 2 elements
    if (range_out.has_value() && range_out.value().size() != 2)
    {
      RCLCPP_ERROR(logger,
                   "Parameter %s.range.out should have exactly two elements, representing the upper and lower bound "
                   "respectively.",
                   prefix.c_str());
      range_in = std::nullopt;
    }

    // Enforce default value for range_in
    if (!range_in.has_value())
      range_in = std::vector<double>{ -1.0, 1.0 };

    range = AxisTransformParams::Range{ std::array<float, 2>{ static_cast<float>(range_in.value()[0]),
                                                              static_cast<float>(range_in.value()[1]) },
                                        std::nullopt, range_clamp };

    if (range_out.has_value())
      range.value().out =
          std::array<float, 2>{ static_cast<float>(range_out.value()[0]), static_cast<float>(range_out.value()[1]) };
  }

  return params;
}

void InputSourceHandle::remap(InputSource::InputDeclarationSpans declarations, RemapParams remap_params)
{
  const auto logger = source_->get_node()->get_logger();
  auto& inputs = inputs_.get();

  // Stores, for each exported button, whether the button is remapped
  std::vector<bool> is_button_remapped(declarations.button_names.size(), false);
  std::vector<bool> is_axis_remapped(declarations.axis_names.size(), false);

  // Reset definitions
  button_definitions_.clear();
  button_definitions_.reserve(declarations.button_names.size());
  transformed_buttons_.clear();
  transformed_buttons_.reserve(remap_params.buttons.size());
  std::vector<Button::SharedPtr> transformed_buttons_deferred_registration;
  transformed_buttons_deferred_registration.reserve(remap_params.buttons.size());

  // Create definitions for each remapped input, marking which ones have been remapped
  auto& buttons = inputs.get_buttons();
  for (auto& [name, from, transform, from_axis] : remap_params.buttons)
  {
    // Find if there is a direct renaming
    std::optional<std::reference_wrapper<uint8_t>> reference = std::nullopt;
    if (from.has_value())
    {  // Find the index for the 'from' param in button_names
      const auto it = std::find(declarations.button_names.begin(), declarations.button_names.end(), from);

      if (it != declarations.button_names.end())
      {
        const size_t index = std::distance(declarations.button_names.begin(), it);
        is_button_remapped[index] = true;

        reference = std::ref(declarations.buttons[index]);
      }
      else
      {
        // TODO: Recursively declare the missing 'from', merging transforms until reaching a valid original input
      }
    }

    // Exit early if we can just register the original memory address under a different name
    const bool needs_extra_memory = transform.has_value() || from_axis.has_value();
    if (!needs_extra_memory)
    {
      button_definitions_.emplace_back(buttons[name], reference.has_value() ?
                                                          (std::vector{ reference.value() }) :
                                                          (std::vector<std::reference_wrapper<uint8_t>>{}));
      continue;
    }

    // Find if there is a remapping from an axis
    std::optional<TransformedRemapButtonFromAxis> from_axis_transform;
    if (from_axis.has_value())
    {
      const auto axis_it =
          std::find(declarations.axis_names.begin(), declarations.axis_names.end(), from_axis.value().name);

      if (axis_it != declarations.axis_names.end())
      {
        const size_t axis_index = std::distance(declarations.axis_names.begin(), axis_it);
        const auto axis_reference = std::ref(declarations.axes[axis_index]);
        from_axis_transform = TransformedRemapButtonFromAxis{ axis_reference, from_axis.value().threshold };
        is_axis_remapped[axis_index] = true;
      }
    }

    // Create the transform, but defer registration to after all the transformed buttons have been created.
    transformed_buttons_.emplace_back(0, reference, from_axis_transform);
    transformed_buttons_deferred_registration.emplace_back(buttons[name]);
  }

  // Register deferred transform buttons
  for (size_t i = 0; i < transformed_buttons_deferred_registration.size(); ++i)
  {
    auto& transformed_button = transformed_buttons_[i];
    const auto& button = transformed_buttons_deferred_registration[i];
    button_definitions_.emplace_back(button, std::vector{ std::ref(transformed_button.value) });
  }

  // Reset definitions
  axis_definitions_.clear();
  axis_definitions_.reserve(declarations.axis_names.size());
  transformed_axes_.clear();
  transformed_axes_.reserve(remap_params.axes.size());
  std::vector<Axis::SharedPtr> transformed_axes_deferred_registration;
  transformed_axes_deferred_registration.reserve(remap_params.axes.size());

  // Create definitions for each remapped input, marking which ones have been remapped
  auto& axes = inputs.get_axes();
  for (auto& [name, from, transform, from_buttons] : remap_params.axes)
  {
    // Find if there is a direct renaming
    std::optional<std::reference_wrapper<float>> reference = std::nullopt;
    if (from.has_value())
    {  // Find the index for the 'from' param in axis_names
      const auto it = std::find(declarations.axis_names.begin(), declarations.axis_names.end(), from);

      if (it != declarations.axis_names.end())
      {
        const size_t index = std::distance(declarations.axis_names.begin(), it);
        is_axis_remapped[index] = true;

        reference = std::ref(declarations.axes[index]);
      }
      else
      {
        // TODO: Recursively declare the missing 'from', merging transforms until reaching a valid original input
      }
    }

    // Exit early if we can just register the original memory address under a different name
    const bool needs_extra_memory = transform.has_value() || from_buttons.has_value();
    if (!needs_extra_memory)
    {
      axis_definitions_.emplace_back(axes[name], reference.has_value() ?
                                                     (std::vector{ reference.value() }) :
                                                     (std::vector<std::reference_wrapper<float>>{}));
      continue;
    }

    // Find if there is a remapping from an axis
    std::optional<TransformedRemapAxisFromButtons> from_buttons_transform;
    if (from_buttons.has_value())
    {
      std::optional<std::reference_wrapper<uint8_t>> negative_reference = std::nullopt;
      if (from_buttons.value().negative.has_value())
      {
        const auto negative_it = std::find(declarations.button_names.begin(), declarations.button_names.end(),
                                           from_buttons.value().negative.value());

        if (negative_it != declarations.button_names.end())
        {
          const size_t negative_index = std::distance(declarations.button_names.begin(), negative_it);
          negative_reference = std::ref(declarations.buttons[negative_index]);
          is_button_remapped[negative_index] = true;
        }
      }

      std::optional<std::reference_wrapper<uint8_t>> positive_reference = std::nullopt;
      if (from_buttons.value().positive.has_value())
      {
        const auto positive_it = std::find(declarations.button_names.begin(), declarations.button_names.end(),
                                           from_buttons.value().positive.value());

        if (positive_it != declarations.button_names.end())
        {
          const size_t positive_index = std::distance(declarations.button_names.begin(), positive_it);
          positive_reference = std::ref(declarations.buttons[positive_index]);
          is_button_remapped[positive_index] = true;
        }
      }

      from_buttons_transform = TransformedRemapAxisFromButtons{ negative_reference, positive_reference };
    }

    // Create the transform, but defer registration to after all the transformed axes have been created.
    transformed_axes_.emplace_back(0.0f, reference, from_buttons_transform);
    transformed_axes_deferred_registration.emplace_back(axes[name]);
  }

  // Register deferred transform axes
  for (size_t i = 0; i < transformed_axes_deferred_registration.size(); ++i)
  {
    auto& transformed_axis = transformed_axes_[i];
    const auto& axis = transformed_axes_deferred_registration[i];
    axis_definitions_.emplace_back(axis, std::vector{ std::ref(transformed_axis.value) });
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
