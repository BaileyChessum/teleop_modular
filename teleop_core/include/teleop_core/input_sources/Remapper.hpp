//
// Created by nova on 7/22/25.
//

#ifndef CONTROL_MODE_REMAPPER_HPP
#define CONTROL_MODE_REMAPPER_HPP

#include "input_source/utilities/span.hpp"
#include <string>
#include <array>
#include <set>
#include <cstdint>
#include <tuple>
#include <rclcpp/logging.hpp>
#include <string_view>
#include <utility>

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

struct ButtonTransformParams
{
};
struct AxisTransformParams
{
};

struct ButtonFromAxisParams
{
};
struct AxisFromButtonParams
{
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
  using from = AxisTransformParams;
};

template<>
struct PairAssoc<uint8_t, float>
{
  using from = AxisTransformParams;
};

template<>
struct PairAssoc<float, float>
{
  using from = AxisTransformParams;
};

template<>
struct PairAssoc<float, uint8_t>
{
  using from = AxisTransformParams;
};

template<typename Type, typename From>
std::string pair_name()
{
  return std::string(Assoc<Type>::name) + " from " + std::string(Assoc<Type>::name);
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

  rclcpp::Logger logger;

  /// Helper to get index of U in T...
  /// Expensive in compile time, please use sparingly.
  template<typename U, std::size_t Index = 0>
  static constexpr std::size_t type_index()
  {
    if constexpr (Index == sizeof...(T)) {
      static_assert(Index != sizeof...(T), "Type U not found in T...");
    } else if constexpr (std::is_same_v<U, std::tuple_element_t<Index, std::tuple<T...>>>) {
      return Index;
    } else {
      return type_index<U, Index + 1>();
    }
  }

  Remapper(
    OriginalDefinitions<T>... originals_,
    std::array<std::set<std::string>, sizeof...(T)> used_name_sets,
    rclcpp::Logger logger)
  : originals(originals_ ...), used_name_sets(used_name_sets), logger(std::move(logger))
  {
  }

  /// All the original definitions exported by the input source
  std::tuple<OriginalDefinitions<T>...> originals;
  /// Helper to get the original for some type T
  template<typename U>
  OriginalDefinitions<U> & original()
  {
    return std::get<OriginalDefinitions<U>>(originals);
  }

  /// For each type T, the set of names actually used by the system consuming inputs.
  std::array<std::set<std::string>, sizeof...(T)> used_name_sets;
  /// Helper to statically index into used_name_sets
  template<std::size_t I>
  std::set<std::string> & used_name_set_at()
  {
    return std::get<I>(used_name_sets);
  }

  // Called for each pair (U = T[I], From = T[J])
  template<typename Type, std::size_t J>
  inline void process_intertype_mapping()
  {
    using From = std::tuple_element_t<J, std::tuple<T...>>;
    static_assert(
      std::is_class_v<typename PairAssoc<Type, From>::from>,
      "Pair is not mappable! Define values for PairAssoc for these types.");
    using Pair = PairAssoc<Type, From>;

    RCLCPP_INFO(
      rclcpp::get_logger("remapper"), "process_pair() for %s",
      pair_name<Type, From>().c_str());

    // Get params for this pair


  }

  /// Applies process_pair<U, J>() for every J
  template<typename U, std::size_t... J>
  inline void process_intertype_mappings_aux(std::index_sequence<J...>)
  {
    // Just run the above method for every J
    (process_intertype_mapping<U, J>(), ...);
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
        logger, "Checking remap for %s %s", std::string(Assoc<Type>::name).c_str(),
        name.c_str());


      // Get mapping for every other type
      process_intertype_mappings_aux<Type>(std::make_index_sequence<sizeof...(T)>{});
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
};

}  // namespace internal
}  // namespace teleop

#endif  // CONTROL_MODE_REMAPPER_HPP
