//
// Created by Bailey Chessum on 7/22/25.
//
// Look, I know. This whole file wishes it were written in Haskell.

#ifndef CONTROL_MODE_REMAPPER_HPP
#define CONTROL_MODE_REMAPPER_HPP

#include "input_source/utilities/span.hpp"
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

namespace teleop
{
namespace internal
{

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
  [[nodiscard]] T* first() const noexcept
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
    if (!value)
      return;

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
};

template<>
struct PairAssoc<uint8_t, float>
{
  using from = ButtonFromAxisParams;
};

template<>
struct PairAssoc<float, float>
{
  using from = DirectReducer<float>;
};

template<>
struct PairAssoc<float, uint8_t>
{
  using from = AxisFromButtonParams;
};

template<typename Type, typename From>
std::string pair_name()
{
  return std::string(Assoc<Type>::name) + " from " + std::string(Assoc<From>::name);
}

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
    OriginalDefinitions<T>... originals_,
    std::array<std::set<std::string>, sizeof...(T)> used_name_sets,
    rclcpp::Logger logger)
  : originals(originals_ ...), used_name_sets(used_name_sets), logger(std::move(logger))
  {
  }

  /// Helper to get the original for some type T
  template<typename U>
  OriginalDefinitions<U> & get_original()
  {
    return std::get<OriginalDefinitions<U>>(originals);
  }
  /// Helper to statically index into originals
  template<size_t I>
  auto & original_at()
  {
    return std::get<I>(originals);
  }


  /// Helper to statically index into used_name_sets
  template<std::size_t I>
  std::set<std::string> & used_name_set_at()
  {
    return std::get<I>(used_name_sets);
  }

  /// Helper to statically index into originals
  template<size_t I>
  auto & transformed_values_at()
  {
    return std::get<I>(transformed_values);
  }

  template<size_t I, typename Type>
  inline void process_same_type_mapping(const std::string & used_name)
  {
    RCLCPP_INFO(logger, C_INPUT "  %s remap" C_RESET, Assoc<Type>::name.data());

    // Check if it is defined in originals
    OriginalDefinitions<Type> original = original_at<I>();
    std::set<std::string> original_names = original.names;

    auto it = original.names.find(used_name);
    if (it != original.names.end()) {}
  }

  /**
   * Gets the reference_wrapper<From>
   */
  template<size_t I, typename Type, std::size_t J>
  inline auto process_intertype_mapping(const std::string & used_name)
  {
    if constexpr (I == J) {


    } else {
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

  }

  /// Applies process_pair<U, J>() for every J
  template<size_t I, typename Type, std::size_t... J>
  inline auto process_intertype_mappings_aux(
    const std::string & used_name,
    std::index_sequence<J...>)
  {
    // Run the above method for every J and put the result in a tuple
    return std::make_tuple(process_intertype_mapping<I, Type, J>(used_name)...);
  }

  template<std::size_t I, typename Type>
  inline Type * get_definition_for(const std::string & used_name)
  {
    process_intertype_mappings_aux<I, Type>(std::make_index_sequence<sizeof...(T)>{});


    return nullptr;
  }

  /**
   * \brief Remap values for the Ith type in typename ... T.
   *
   * \tparam I  The index of the type we are mapping values to in typename ... T
   */
  template<std::size_t I>
  inline void remap_inputs_of_type()
  {
    using Type = std::tuple_element_t<I, std::tuple<T...>>;

    // Get names that control modes actually use
    const auto used_names = used_name_set_at<I>();

    for (auto & name : used_names) {
      RCLCPP_INFO(
        logger, "Checking remap for %s " C_INPUT "%s" C_RESET, Assoc<Type>::name.data(),
        name.c_str());

      // Get mapping for every other type
    }
  }

  // Outer loop over all I
  template<std::size_t... I>
  inline void process_all_types_aux(std::index_sequence<I...>)
  {
    (remap_inputs_of_type<I>(), ...);
  }

  inline void process_all_types()
  {
    process_all_types_aux(std::make_index_sequence<sizeof...(T)>{});
  }

  rclcpp::Logger logger;
  /// All the original definitions exported by the input source
  std::tuple<OriginalDefinitions<T>...> originals;
  /// For each type T, the set of names actually used by the system consuming inputs.
  std::array<std::set<std::string>, sizeof...(T)> used_name_sets;
  /// For each type T, additional transformed values from complex remappings
  std::tuple<std::vector<TransformedValue<T>>...> transformed_values;
};

}  // namespace internal
}  // namespace teleop

#endif  // CONTROL_MODE_REMAPPER_HPP
