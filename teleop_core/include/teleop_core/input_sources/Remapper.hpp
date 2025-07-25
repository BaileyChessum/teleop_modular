//
// Created by Bailey Chessum on 7/22/25.
//
// Look, I know. This whole file wishes it were written in Haskell.

#ifndef CONTROL_MODE_REMAPPER_HPP
#define CONTROL_MODE_REMAPPER_HPP

#include "input_source/utilities/span.hpp"
#include "input_source/utilities/vector_ref.hpp"
#include "teleop_core/colors.hpp"
#include <string>
#include <array>
#include <set>
#include <cstdint>
#include <tuple>
#include <rclcpp/logging.hpp>
#include <string_view>
#include <utility>
#include <vector>
#include <memory>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_parameters.hpp>
#include "teleop_core/utilities/get_parameter.hpp"

namespace teleop
{
namespace internal::remapping
{

using ParametersInterface = rclcpp::node_interfaces::NodeParametersInterface;

// To get span<T> without the prefix
using namespace input_source;

/**
 * \brief Provided by the input source, allows us to see the raw set of values exported by an input source.
 *
 * \tparam T    The basic type held by the input type (float, uint8_t, etc)
 */
template<typename T>
struct OriginalDefinitions
{
  const span<std::string> names;
  const span<T> values;
};


template<typename T>
struct Transformer
{
  virtual void accumulate(T & result) noexcept = 0;
};

/**
 * Either something that reduces multiple original inputs into a single result, or a transform
 */
template<typename Type, typename From>
struct Reducer : Transformer<Type>
{
  std::vector<From *> values;

  [[nodiscard]] bool empty() const noexcept
  {
    return values.empty();
  }

  [[nodiscard]] size_t size() const noexcept
  {
    return values.size();
  }
};

template<typename T>
struct DirectReducer final : public Reducer<T, T>
{
  std::vector<T *> values;

  inline void accumulate(uint8_t & result) noexcept final
  {
    for (size_t i = 0; i < values.size(); ++i) {
      result += *values[i];
    }
  }

  /// Special case for DirectReducers to allow direct mapping
  [[nodiscard]] T * first() const noexcept
  {
    assert(this->size() == 1);
    return values[0];
  }
};


/// What we actually store in memory for transformed things
template<typename T>
struct TransformedValue final
{
  T value;
  const std::vector<std::shared_ptr<Transformer<T>>> transformers;

  inline void update() noexcept
  {
    value = 0;
    for (const auto & transformer : transformers) {
      transformer->accumulate(value);
    }
  }

  explicit TransformedValue(std::vector<std::shared_ptr<Transformer<T>>> transformers)
  : transformers(std::move(transformers))
  {
    value = 0;
  }
};

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
 * The result of some part of the parameterization stage
 */
template<typename ParamT>
struct ParamDefinitions
{
  std::vector<std::string> names;
  std::vector<ParamT> params;
};

//region Assoc

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

  static std::vector<std::string> get_params(
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

    if (from.has_value())
      from_any.emplace_back(from.value());

    return from_any;
  }
};

template<>
struct PairAssoc<uint8_t, float>
{
  using from = ButtonFromAxisParams;
  using params = float;

  static ParamDefinitions<params> get_params(
    const ParametersInterface::SharedPtr & interface,
    const std::string & used_name)
  {
    ParamDefinitions<params> result;




    return result;
  }
};

template<>
struct PairAssoc<float, float>
{
  using from = DirectReducer<float>;

  static std::vector<std::string> get_params(
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

    if (from.has_value())
      from_any.emplace_back(from.value());

    return from_any;
  }
};

template<>
struct PairAssoc<float, uint8_t>
{
  using from = AxisFromButtonParams;
  using params = float;
};

//endregion

/**
 * The inputs given by the external system to run the algorithm on
 */
template<typename ... T>
struct AlgorithmInputs{

  std::tuple<OriginalDefinitions<T>...> originals;

};



template<typename ... T>
struct ParamPhaseData
{
  struct LookupEntry {
    size_t index{};
    bool original = false;
  };

  /// The original inputs to the algorithm
  AlgorithmInputs<T...> inputs;

  /// Maps name to index and info to identify the appropriate container

  std::array<std::map<std::string, LookupEntry>, sizeof...(T)> lookup{};
  size_t new_entries = 0;

  template<std::size_t I>
  inline void constructor_foreach_type() {
    auto og_names = std::get<I>(inputs.originals).names;
    for (size_t j = 0; j < og_names.size(); ++j)
      lookup[I][og_names[j]] = LookupEntry{j, true};
  }

  template<std::size_t... I>
  inline void constructor_foreach_type_aux(std::index_sequence<I...>)
  {
    (constructor_foreach_type<I>(), ...);
  }

  explicit ParamPhaseData(AlgorithmInputs<T...> inputs) : inputs(inputs)
  {
    constructor_foreach_type_aux(std::make_index_sequence<sizeof...(T)>{});
  }

//  size_t make_entry_for(const std::string& new_name) {
//    value_lookup[new_name] = LookupEntry{new_entries, true};
//    return new_entries++;
//  }
};


/**
 * \brief A templated class to reduce duplicated logic for input source remapping
 *
 * \tparam T    The basic type held by the input source, that can be mapped to each other (float, uint8_t, etc)
 */
template<typename ... T>
class Remapper
{
public:
  static_assert(
    (std::is_class_v<typename Assoc<T>::transforms>&& ...),
    "All types must be mappable via Assoc");

  Remapper(
    rclcpp::Logger logger, ParametersInterface::SharedPtr interface)
  : logger(std::move(logger)),
    interface(std::move(interface))
  {
  }

  /// Helper to statically index into originals
  template<size_t I>
  auto & transformed_values_at()
  {
    return std::get<I>(transformed_values);
  }

  template<size_t I, typename Type>
  inline void param_phase_same_type_mapping(const std::string & used_name)
  {
    RCLCPP_INFO(logger, C_INPUT "  %s remap" C_RESET, Assoc<Type>::name.data());

    auto reducer = std::make_shared<DirectReducer<Type>>();

    // Check if it is defined in originals
//    OriginalDefinitions<Type> original = original_at<I>();
//    span<std::string> original_names = original.names;
//    const auto it = std::find(original_names.begin(), original_names.end(), used_name);
//
//    if (it != original.names.end()) {
//      const size_t original_index = std::distance(original_names.begin(), it);
//      reducer->values.emplace_back(original.values[original_index]);
//    }
  }

  /**
   * Gets the reference_wrapper<From>
   */
  template<size_t I, typename Type, std::size_t J>
  inline auto param_phase_intertype_mapping(const std::string & used_name)
  {
    if constexpr (I == J) {
      // Direct reduction
      return param_phase_same_type_mapping<I, Type>();
    }

    using From = std::tuple_element_t<J, std::tuple<T...>>;
    static_assert(
      std::is_class_v<typename PairAssoc<Type, From>::from>,
      "Pair is not mappable! Define values for PairAssoc for these types.");
    using Pair = PairAssoc<Type, From>;

    // Get params for this pair
    typename Pair::from result;
    return result;

    RCLCPP_INFO(
      logger, C_INPUT "  %s from %s" C_RESET, Assoc<Type>::name.data(),
      Assoc<From>::name.data());
  }

  /// Applies process_pair<U, J>() for every J
  template<size_t I, typename Type, std::size_t... J>
  inline auto param_phase_intertype_mappings_aux(
    const std::string & used_name,
    std::index_sequence<J...>)
  {
    // Run the above method for every J and put the result in a tuple
    return std::make_tuple(param_phase_intertype_mapping<I, Type, J>(used_name)...);
  }

  template<std::size_t I, typename Type>
  inline Type * param_phase_get_definition_for(const std::string & used_name)
  {
    param_phase_intertype_mappings_aux<I, Type>(std::make_index_sequence<sizeof...(T)>{});


    return nullptr;
  }

  /**
   * \brief Remap values for the Ith type in typename ... T.
   *
   * \tparam I  The index of the type we are mapping values to in typename ... T
   */
  template<std::size_t I>
  inline void param_phase_remap_inputs_of_type(ParamPhaseData<T...>& data)
  {
    using Type = std::tuple_element_t<I, std::tuple<T...>>;

    // Get names that control modes actually use

//    for (auto & name : used_names) {
//      RCLCPP_INFO(
//        logger, "Checking remap for %s " C_INPUT "%s" C_RESET, Assoc<Type>::name.data(),
//        name.c_str());
//
//      // Get mapping for every other type
//    }
  }

  // Outer loop over all I
  template<std::size_t... I>
  inline void param_phase_all_types_aux(ParamPhaseData<T...>& data, std::index_sequence<I...>)
  {
    (param_phase_remap_inputs_of_type<I>(data), ...);
  }

  inline ParamPhaseData<T...> param_phase_all_types(AlgorithmInputs<T...> inputs)
  {
    ParamPhaseData<T...> data = ParamPhaseData<T...>(inputs);

    param_phase_all_types_aux(data, std::make_index_sequence<sizeof...(T)>{});
  }

  rclcpp::Logger logger;
  ParametersInterface::SharedPtr interface;
  /// All the original definitions exported by the input source
//  std::tuple<OriginalDefinitions<T>...> originals;
  /// For each type T, the set of names actually used by the system consuming inputs.
//  std::array<std::set<std::string>, sizeof...(T)> used_name_sets;
  /// For each type T, additional transformed values from complex remappings
  std::tuple<std::vector<std::shared_ptr<TransformedValue<T>>>...> transformed_values;
};

}  // namespace internal
}  // namespace teleop

#endif  // CONTROL_MODE_REMAPPER_HPP
