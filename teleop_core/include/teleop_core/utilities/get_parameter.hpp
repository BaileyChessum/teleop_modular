//
// Created by nova on 7/7/25.
//

#ifndef TELEOP_MODULAR_GET_PARAMETER_HPP
#define TELEOP_MODULAR_GET_PARAMETER_HPP

#include <optional>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace teleop::utils
{

template<typename T>
inline std::optional<T> get_parameter(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
  const std::string & name, const std::string & description);

template<typename T>
inline T get_parameter_or_default(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
  const std::string & name, const std::string & description, T default_value)
{
  auto value = get_parameter<T>(parameters, name, description);
  return value.has_value() ? value.value() : default_value;
}

template<>
inline std::optional<bool> get_parameter(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
  const std::string & name, const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor from_descriptor;
  from_descriptor.name = name;
  from_descriptor.description = description;
  parameters->declare_parameter(from_descriptor.name, rclcpp::PARAMETER_BOOL, from_descriptor);

  rclcpp::Parameter from_param;
  auto type_result = parameters->get_parameter(from_descriptor.name, from_param);

  if (!type_result || from_param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
    return std::nullopt;
  }

  return from_param.as_bool();
}

template<>
inline std::optional<std::vector<bool>>
get_parameter(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
  const std::string & name,
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor from_descriptor;
  from_descriptor.name = name;
  from_descriptor.description = description;
  parameters->declare_parameter(
    from_descriptor.name, rclcpp::PARAMETER_BOOL_ARRAY,
    from_descriptor);

  rclcpp::Parameter from_param;
  auto type_result = parameters->get_parameter(from_descriptor.name, from_param);

  if (!type_result || from_param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL_ARRAY) {
    return std::nullopt;
  }

  return from_param.as_bool_array();
}

template<>
inline std::optional<double>
get_parameter(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
  const std::string & name,
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor from_descriptor;
  from_descriptor.name = name;
  from_descriptor.description = description;
  parameters->declare_parameter(from_descriptor.name, rclcpp::PARAMETER_DOUBLE, from_descriptor);

  rclcpp::Parameter from_param;
  auto type_result = parameters->get_parameter(from_descriptor.name, from_param);

  if (!type_result || from_param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return std::nullopt;
  }

  return from_param.as_double();
}

template<>
inline std::optional<std::vector<double>>
get_parameter(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
  const std::string & name,
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor from_descriptor;
  from_descriptor.name = name;
  from_descriptor.description = description;
  parameters->declare_parameter(
    from_descriptor.name, rclcpp::PARAMETER_DOUBLE_ARRAY,
    from_descriptor);

  rclcpp::Parameter from_param;
  auto type_result = parameters->get_parameter(from_descriptor.name, from_param);

  if (!type_result || from_param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    return std::nullopt;
  }

  return from_param.as_double_array();
}

template<>
inline std::optional<float> get_parameter(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
  const std::string & name, const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor from_descriptor;
  from_descriptor.name = name;
  from_descriptor.description = description;
  parameters->declare_parameter(from_descriptor.name, rclcpp::PARAMETER_DOUBLE, from_descriptor);

  rclcpp::Parameter from_param;
  auto type_result = parameters->get_parameter(from_descriptor.name, from_param);

  if (!type_result || from_param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return std::nullopt;
  }

  return static_cast<float>(from_param.as_double());
}

template<>
inline std::optional<std::string>
get_parameter(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
  const std::string & name,
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor from_descriptor;
  from_descriptor.name = name;
  from_descriptor.description = description;
  parameters->declare_parameter(from_descriptor.name, rclcpp::PARAMETER_STRING, from_descriptor);

  rclcpp::Parameter from_param;
  auto type_result = parameters->get_parameter(from_descriptor.name, from_param);

  if (!type_result || from_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    return std::nullopt;
  }

  return from_param.as_string();
}

template<>
inline std::optional<std::vector<std::string>>
get_parameter(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
  const std::string & name,
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor from_descriptor;
  from_descriptor.name = name;
  from_descriptor.description = description;
  parameters->declare_parameter(
    from_descriptor.name, rclcpp::PARAMETER_STRING_ARRAY,
    from_descriptor);

  rclcpp::Parameter from_param;
  auto type_result = parameters->get_parameter(from_descriptor.name, from_param);

  if (!type_result || from_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    return std::nullopt;
  }

  return from_param.as_string_array();
}

template<>
inline std::optional<int64_t>
get_parameter(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
  const std::string & name,
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor from_descriptor;
  from_descriptor.name = name;
  from_descriptor.description = description;
  parameters->declare_parameter(from_descriptor.name, rclcpp::PARAMETER_INTEGER, from_descriptor);

  rclcpp::Parameter from_param;
  auto type_result = parameters->get_parameter(from_descriptor.name, from_param);

  if (!type_result || from_param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
    return std::nullopt;
  }

  return from_param.as_int();
}

template<>
inline std::optional<std::vector<int64_t>>
get_parameter(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
  const std::string & name,
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor from_descriptor;
  from_descriptor.name = name;
  from_descriptor.description = description;
  parameters->declare_parameter(
    from_descriptor.name, rclcpp::PARAMETER_INTEGER_ARRAY,
    from_descriptor);

  rclcpp::Parameter from_param;
  auto type_result = parameters->get_parameter(from_descriptor.name, from_param);

  if (!type_result || from_param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    return std::nullopt;
  }

  return from_param.as_integer_array();
}

}  // namespace teleop::utils

#endif  // TELEOP_MODULAR_GET_PARAMETER_HPP
