//
// Created by nova on 7/25/25.
//

#ifndef CONTROL_MODE_REMAPPER_ASSOC_HPP
#define CONTROL_MODE_REMAPPER_ASSOC_HPP

#include "teleop_core/input_sources/remapper_reducer.hpp"
#include <rclcpp/node_interfaces/node_parameters.hpp>
#include <string_view>
#include "teleop_core/utilities/get_parameter.hpp"
#include <string>
#include <teleop_core/utilities/get_parameter.hpp>

namespace teleop::internal::remapping
{

using ParametersInterface = rclcpp::node_interfaces::NodeParametersInterface;

struct LookupEntry
{
  size_t index{};
  bool original = false;
};

/**
 * The result of some part of the parameterization stage
 */
template<typename ParamT>
struct ParamDefinitions
{
  std::vector<std::string> names;
  std::vector<ParamT> params;
};

using DirectParamDefinitions = ParamDefinitions<bool>;

struct AxisTransformParams final : public Transformer<float>
{
  inline void accumulate(float & result) noexcept final
  {
    // TODO(BaileyChessum): Do transformation stuff here
  }
};

struct ButtonTransformParams final : public Transformer<uint8_t>
{
  inline void accumulate(uint8_t & result) noexcept final
  {
    // TODO(BaileyChessum): Do transformation stuff here
  }
};

struct AxisFromButtonParams final : public Reducer<float, uint8_t>
{
  std::vector<uint8_t *> values;
  /// When the value is true, the equivalent will be added to the output. 1.0 for positive, -1.0 for negative buttons.
  std::vector<float> equivalents;


  void push_back(uint8_t * value, bool negative)
  {
    if (!value) {
      return;
    }

    values.emplace_back(value);
    equivalents.emplace_back(negative ? -1.0f : 1.0f);
  }

  inline void accumulate(float & result) noexcept final
  {
    for (size_t i = 0; i < values.size(); ++i) {
      result += static_cast<float>(*values[i]) * equivalents[i];
    }
  }
};

struct ButtonFromAxisParams final : public Reducer<uint8_t, float>
{
  std::vector<float> thresholds;

  void push_back(float * value, float threshold)
  {
    if (!value) {
      return;
    }

    values.emplace_back(value);
    thresholds.emplace_back(threshold);
  }

  inline void accumulate(uint8_t & result) noexcept final
  {
    for (size_t i = 0; i < values.size(); ++i) {
      if (*values[i] < thresholds[i]) {
        ++result;
      }
    }
  }
};

/**
 * \breif helper to associate various extra types with T
 * We use this to associate transform params with T
 */
template<typename T, typename = void>
struct Assoc;  // Defined per T

// Map Params for different types
template<>
struct Assoc<uint8_t>
{
  static constexpr const std::string_view name = "Button";
  using transforms = ButtonTransformParams;
};

template<>
struct Assoc<float>
{
  static constexpr const std::string_view name = "Axis";
  using transforms = AxisTransformParams;
};

/**
 * \breif helper to associate how one type relates to another
 * We use this to define from params in remapping, such as making a button from an axis
 */
template<typename T, typename TFrom, typename = void>
struct PairAssoc;  // Defined per T

template<>
struct PairAssoc<uint8_t, uint8_t>
{
  using from = DirectReducer<uint8_t>;
  // TODO(BaileyChessum): Direct mappings don't actually use this -- should be void
  using params = bool;
  using pre_param = std::pair<std::vector<LookupEntry>, std::vector<params>>;

  static DirectParamDefinitions get_params(
    const ParametersInterface::SharedPtr & interface,
    const std::string & used_name)
  {
    auto from_any = utils::get_parameter_or_default<std::vector<std::string>>(
      interface,
      "remap.buttons." + used_name + ".from_any",
      "The original button names to derive values for this name from.", {});
    const auto from = utils::get_parameter<std::string>(
      interface,
      "remap.buttons." + used_name + ".from",
      "The original button name to derive values for this name from.");

    if (from.has_value()) {
      from_any.emplace_back(from.value());
    }

    return {from_any, {}};
  }
};

template<>
struct PairAssoc<uint8_t, float>
{
  using from = ButtonFromAxisParams;
  using params = float;
  using pre_param = std::pair<std::vector<LookupEntry>, std::vector<params>>;

  static ParamDefinitions<params> get_params(
    const ParametersInterface::SharedPtr & interface,
    const std::string & used_name)
  {
    ParamDefinitions<params> result;

    const auto prefix = "remap.axes." + used_name + ".from_buttons.";

    // Get all negative froms
    std::vector<std::string> negatives = utils::get_parameter_or_default(
        interface, prefix + "negative",
        "A button that when pressed will cause the axis to be negative.", std::vector<std::string>());
    const auto from_button_negative = utils::get_parameter<std::string>(
        interface, prefix + "negatives",
        "Buttons that when pressed will cause the axis to be negative.");
    if (from_button_negative.has_value())
      negatives.push_back(from_button_negative.value());

    // Get all positive froms
    const auto from_button_positive = utils::get_parameter<std::string>(
        interface, prefix + "positive",
        "A button that when pressed will cause the axis to be positive.");
    std::vector<std::string> positives = utils::get_parameter_or_default(
        interface, prefix + "positives",
        "Buttons that when pressed will cause the axis to be positive.", std::vector<std::string>());
    if (from_button_positive.has_value())
      positives.push_back(from_button_positive.value());

    // Collate into the two arrays
    size_t count = negatives.size() + positives.size();
    if (count == 0)
      return result;  //< Most mappings will be empty

    result.names = positives;
    result.names.reserve(count);
    result.params.reserve(count);
    for (size_t i = 0; i < positives.size(); i++) {
      result.params.emplace_back(1.0f);
    }

    for (size_t i = 0; i < negatives.size(); i++) {
      result.names.push_back(negatives[i]);
      result.params.emplace_back(-1.0f);
    }

    return result;
  }
};

template<>
struct PairAssoc<float, float>
{
  using from = DirectReducer<float>;
  // TODO(BaileyChessum): Direct mappings don't actually use this -- should be void
  using params = bool;
  using pre_param = std::pair<std::vector<LookupEntry>, std::vector<params>>;

  static DirectParamDefinitions get_params(
    const ParametersInterface::SharedPtr & interface,
    const std::string & used_name)
  {
    auto from_any = utils::get_parameter_or_default<std::vector<std::string>>(
      interface,
      "remap.axes." + used_name + ".from_any",
      "The original axis names to derive values for this name from.", {});
    const auto from = utils::get_parameter<std::string>(
      interface,
      "remap.axes." + used_name + ".from",
      "The original axis name to derive values for this name from.");

    if (from.has_value()) {
      from_any.emplace_back(from.value());
    }

    return {from_any, {}};
  }
};

template<>
struct PairAssoc<float, uint8_t>
{
  using from = AxisFromButtonParams;
  using params = float;
  using pre_param = std::pair<std::vector<LookupEntry>, std::vector<params>>;

  static ParamDefinitions<params> get_params(
    const ParametersInterface::SharedPtr & interface,
    const std::string & used_name)
  {
    ParamDefinitions<params> result;
    const auto prefix = "remap.buttons." + used_name + ".from_axis.";

    const auto from_axis_name = utils::get_parameter<std::string>(
        interface, prefix + "name",
        "The name of the axis to read the button value from.");

    if (!from_axis_name.has_value())
      return result;

    const auto threshold = utils::get_parameter_or_default<float>(
        interface, prefix + "threshold",
        "The threshold below which the button will be triggered.", 0.0);

    result.names = {from_axis_name.value()};
    result.params = {threshold};

    return result;
  }
};

}  // namespace teleop::internal::remapping


#endif  // CONTROL_MODE_REMAPPER_ASSOC_HPP
