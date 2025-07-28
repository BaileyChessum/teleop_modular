//
// Created by Bailey Chessum on 22/7/25.
//
// Look, I know. This whole file wishes it were written in Haskell.
//
// I tried to do some metaprogramming to make the remapping less terse and more extensible, while keeping good cache
// locality, avoiding excessive indirection, and reducing duplicate code. Since the previous simpler implementation was
// already very error-prone due to its size and duplicated code, I went about this implementation to avoid those issues.
//
// Unfortunately, most of this has to be achieved through complex templates in C++, hence why everything is near
// incomprehensible.
//
// The algorithm is split up into two phases -- the parameterization phase, and crystallization phase
//
// The parameterization phase does the brunt of the work, figuring out for each input type pairing, what inputs relate
// to each other, associated parameters with each pairing, and a LookupEntry object to be used to link things in the
// next phase.
//
// The crystallization phase turns those parameters and LookupEntry objects into Transformers/Reducers and raw pointers.

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
#include "teleop_core/input_sources/remapper_reducer.hpp"
#include "teleop_core/input_sources/remapper_assoc.hpp"
#include "teleop_core/utilities/make_array.hpp"

namespace teleop::internal::remapping
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

/**
 * The inputs given by the external system to run the algorithm on
 *
 * \tparam T    All basic types held by the input source, that can be mapped to each other (float, uint8_t, etc)
 */
template<typename ... T>
struct AlgorithmInputs
{
  std::tuple<OriginalDefinitions<T>...> originals;
  std::array<std::vector<std::string>, sizeof...(T)> used_names;
};

// Cartesian product metafunction
template<typename A, typename ... B>
struct PairWithEach
{
  using pre_param = std::tuple<typename PairAssoc<A, B>::pre_param...>;
  using transformed_params = std::tuple<TransformedParamDefinitions<typename PairAssoc<A, B>::params>...>;
};

template<typename ... T>
struct ParamPhaseData
{
  /// The original inputs to the algorithm
  AlgorithmInputs<T...> inputs;

  /// Maps name to index and info to identify the appropriate container
  std::array<std::map<std::string, LookupEntry>, sizeof...(T)> lookup{};

  std::array<size_t, sizeof...(T)> transformed_value_counts{};

  // Expand over all T[i]
  using pre_param_tuple_type = std::tuple<std::map<std::string, typename PairWithEach<T,
      T...>::pre_param>...>;
  pre_param_tuple_type pre_params;

  using transformed_params_tuple_type = std::tuple<std::vector<typename PairWithEach<T, T...>::transformed_params>...>;
  transformed_params_tuple_type transformed_params;

  template<std::size_t I>
  inline void constructor_foreach_type()
  {
    auto og_names = std::get<I>(inputs.originals).names;
    for (size_t j = 0; j < og_names.size(); ++j) {
      lookup[I][og_names[j]] = LookupEntry{j, true};
    }
  }

  template<std::size_t... I>
  inline void constructor_foreach_type_aux(std::index_sequence<I...>)
  {
    (constructor_foreach_type<I>(), ...);
  }

  explicit ParamPhaseData(AlgorithmInputs<T...> inputs)
  : inputs(inputs)
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
 * \tparam T    All basic types held by the input source, that can be mapped to each other (float, uint8_t, etc)
 */
template<typename ... T>
class Remapper final
{
public:
  static_assert(
    (std::is_class_v<typename Assoc<T>::transforms>&& ...),
    "All types must be mappable via Assoc");

  rclcpp::Logger logger;
  ParametersInterface::SharedPtr interface;
  /// All the original definitions exported by the input source
  //  std::tuple<OriginalDefinitions<T>...> originals;
  /// For each type T, the set of names actually used by the system consuming inputs.
  //  std::array<std::set<std::string>, sizeof...(T)> used_name_sets;

  // TODO(BaileyChessum): Replace with ECS style system (very complex with intertype dependencies -- but doable).
  // Implement if there are performance issues in update cycles in comparison to using only direct mappings
  /// For each type T, additional transformed values from complex remappings
  std::tuple<std::vector<std::shared_ptr<TransformedValue<T>>>...> transformed_values;



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

  template<std::size_t I, typename Type>
  inline bool param_phase_try_make_exist(
    ParamPhaseData<T...> & data,
    const std::string & used_name);

  template<std::size_t I, typename Type>
  inline bool param_phase_ensure_exists(ParamPhaseData<T...> & data, const std::string & used_name);

  /// Needs to find the list of all LookupEntry + params for a certain type pair
  /// Has the responsibility to set pre_params for used_name under this type pair
  /// Return the number of mapped values
  template<std::size_t I, typename Type, std::size_t J>
  inline size_t get_params_for_pair_to_used_name(
    ParamPhaseData<T...> & data,
    const std::string & used_name)
  {
    using From = std::tuple_element_t<J, std::tuple<T...>>;
    using Pair = PairAssoc<Type, From>;

    auto & pre_params_for_type = std::get<I>(data.pre_params);

    typename PairWithEach<Type, T...>::pre_param default_pre_params{};
    auto pre_param_lookup_result = pre_params_for_type.emplace(used_name, default_pre_params); // ,
    // TODO(BaileyChessum): Idk how to get this from the above call
    auto & pre_params_for_pair = std::get<J>(pre_params_for_type[used_name]); //std::get<J>(*(pre_param_lookup_result.first));

    RCLCPP_INFO(
      logger, "Getting all " C_INPUT "%s" C_RESET " inputs to " C_INPUT "%s '%s'" C_RESET,
      Assoc<From>::name.data(), Assoc<Type>::name.data(), used_name.c_str());

    // Same type
    std::vector<std::string> map_to_names{};
    // TODO(BaileyChessum): Direct mappings don't actually use this
    std::vector<typename Pair::params> map_to_params{};

    auto & lookup = data.lookup[J];
    std::vector<LookupEntry> & entries = pre_params_for_pair.first;
    std::vector<typename Pair::params> & params = pre_params_for_pair.second;

    // Populate map_to_names
    if constexpr (I == J) {
      // Mapping between same type, no extra params
      map_to_names = Pair::get_params(interface, used_name).names;
    } else {
      auto pair_params = Pair::get_params(interface, used_name);
      map_to_names = pair_params.names;
      map_to_params = pair_params.params;
    }

    for (size_t i = 0; i < map_to_names.size(); ++i) {
      std::string name = map_to_names[i];
      typename Pair::params param = map_to_params[i];

      // TODO(BaileyChessum): Ensure the name exists, then add if exists
      auto it = lookup.find(name);

      if (it == lookup.end()) {
        // The given name doesn't exist yet -- try make it exist
        if (!param_phase_ensure_exists<J, From>(data, name)) {
          continue;
        }
        // I don't think that this should ever fail? We could change the return type of the above to give us
        // it->second conditionally as a std::optional, where std::nullopt means it failed
        it = lookup.find(name);
      }

      if (it == lookup.end()) {
        continue;
      }

      entries.emplace_back(it->second);
      RCLCPP_INFO(
        logger, "  Found " C_INPUT "%s" C_RESET " at " C_INPUT "%lu, %d" C_RESET,
        name.c_str(), it->second.index, it->second.original);
      if constexpr (I != J) {
        // Types are different
        params.emplace_back(param);
      }
    }

    return entries.size();
  }

  template<std::size_t I, typename Type, std::size_t... J>
  inline std::array<size_t, sizeof...(T)> get_all_params_to_used_name(
    ParamPhaseData<T...> & data,
    const std::string & used_name, std::index_sequence<J...>)
  {
     // TODO: Turn back to normal
     return utils::make_array(get_params_for_pair_to_used_name<I, Type, J>(data, used_name)...);
  }

  template<std::size_t I, typename Type>
  inline bool param_phase_create_leaves_for_type(ParamPhaseData<T...> & data)
  {
    using Pair = PairAssoc<Type, Type>;

    auto & inputs = data.inputs;
    auto & lookup = data.lookup[I];
    auto & originals = std::get<I>(inputs.originals);

    std::vector<std::string> overwritten_original_names{};

    for (size_t i = 0; i < originals.names.size(); ++i) {
      const auto & name = originals.names[i];

      // Try create a leaf for this name
      auto direct_names = Pair::get_params(interface, name);

      if (!direct_names.empty() && !(direct_names.size() == 1 && direct_names[0] == name)) {
        // Uh oh -- this isn't direct :(
        overwritten_original_names.emplace_back(name);
        RCLCPP_WARN(
          logger, "%s is remapped, which is currently undefined behaviour!",
          name.c_str());
        continue;
      }

      // This is done implicitly:
      // lookup[name] = {i, true};
    }

    // TODO(BaileyChessum): Deal with overwritten original names
    return true;
  }

  /**
   * \brief Remap values for the Ith type in typename ... T.
   *
   * \tparam I  The index of the type we are mapping values to in typename ... T
   */
  template<std::size_t I>
  inline void param_phase_remap_inputs_of_type(ParamPhaseData<T...> & data)
  {
    using Type = std::tuple_element_t<I, std::tuple<T...>>;

    std::vector<std::string> & used_names = data.inputs.used_names[I];

    for (auto & used_name : used_names) {
      RCLCPP_INFO(logger, "Remapping %s %s", Assoc<Type>::name.data(), used_name.c_str());
      param_phase_ensure_exists<I, Type>(data, used_name);
    }
  }

  // Outer loop over all I
  template<std::size_t... I>
  inline void param_phase_all_types_aux(ParamPhaseData<T...> & data, std::index_sequence<I...>)
  {
    (param_phase_remap_inputs_of_type<I>(data), ...);
  }

  inline ParamPhaseData<T...> param_phase(AlgorithmInputs<T...> inputs)
  {
    ParamPhaseData<T...> data = ParamPhaseData<T...>(inputs);
    param_phase_all_types_aux(data, std::make_index_sequence<sizeof...(T)>{});
    return data;
  }

  template<std::size_t... I>
  inline void crystallization_resize_transformed_values(ParamPhaseData<T...> & params, std::index_sequence<I...>) {
    // For each I, resize transformed_values to params.transformed_value_counts
    (std::get<I>(transformed_values.resize(std::get<I>(params.transformed_value_counts))), ...);
  }

  inline void crystallization_phase(ParamPhaseData<T...> & params) {
    // Set up vectors based on params
    crystallization_resize_transformed_values(params, std::make_index_sequence<sizeof...(T)>{});

    // Set up 5

  }

  /// Does all steps to remap inputs to be able to start using update()
  inline void remap(AlgorithmInputs<T...> inputs)
  {
    auto start = std::chrono::high_resolution_clock::now();

    ParamPhaseData<T...> param_phase_data = param_phase(inputs);
    crystallization_phase(param_phase_data);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    RCLCPP_INFO(logger, "Remapped parameters in %ld microseconds", duration);
  }
};

template<typename ... T>
template<std::size_t I, typename Type>
bool Remapper<T...>::param_phase_ensure_exists(
  ParamPhaseData<T...> & data,
  const std::string & used_name)
{
  auto & inputs = data.inputs;
  auto & lookup = data.lookup[I];

  auto it = lookup.find(used_name);

  if (it != lookup.end()) {
    auto & entry = *it;

    // Already exists
    return true;
  }

  // Try make it exist
  return param_phase_try_make_exist<I, Type>(data, used_name);
}

template<typename ... T>
template<std::size_t I, typename Type>
bool Remapper<T...>::param_phase_try_make_exist(
  ParamPhaseData<T...> & data,
  const std::string & used_name)
{
  auto & inputs = data.inputs;
  auto & originals = std::get<I>(inputs.originals);

  // The number of mapped values for each 'From' type
  auto params_results = get_all_params_to_used_name<I, Type>(
    data, used_name,
    std::make_index_sequence<sizeof...(T)>{});
  bool has_any_def = false;
  bool needs_transform_value = false;

  // Get values for has_any_def and needs_transform_value
  for (size_t i = 0; i < params_results.size(); ++i) {
    if (i == I) {
      if (params_results[i] > 0) {
        has_any_def = true;
        // Special case when mapping between the same type --
        // we only need a transform value when there are more than 2 source values
        if (params_results[i] > 1) {
          needs_transform_value = true;
        }
      }
    } else {
      if (params_results[i] > 0) {
        has_any_def = true;
        needs_transform_value = true;
      }
    }
  }

  if (!has_any_def) {
    return false;
  }

  LookupEntry entry{};

  if (needs_transform_value) {
    // Create a new transformed value in memory (actual creation is deferred until the crystallization phase)
    entry.index = data.transformed_value_counts[I]++;
    entry.original = false;   // Mark that the crystallized pointer should point into the transformed vector.

  } else {
    // Find the single direct LookupEntry and copy that
    // TODO(BaileyChessum): Duplicated from get_params_for_pair_to_used_name -- collate to avoid duplicate lookup
    auto & pre_params_for_type = std::get<I>(data.pre_params);
    // auto & pre_params_for_pair = std::get<I>(pre_params_for_type[used_name]); //std::get<J>(*(pre_param_lookup_result.first));
    auto & pre_params_for_pair = std::get<I>(pre_params_for_type[used_name]); //std::get<J>(*(pre_param_lookup_result.first));

    // TODO: Do we need to emplace here??
    typename PairWithEach<Type, T...>::pre_param default_pre_params{};
    auto pre_param_lookup_result = pre_params_for_type.emplace(used_name, default_pre_params); // ,

    auto & pre_params_for_direct_mapping = std::get<I>(pre_params_for_type[used_name]);
    std::vector<LookupEntry> & direct_mapping_entries = pre_params_for_direct_mapping.first;

    assert(direct_mapping_entries.size() == 1);
    entry = direct_mapping_entries[0];
  }

  auto & lookup = data.lookup[I];
  lookup[used_name] = entry;

  return true;
}

}  // namespace teleop::internal::remapping


#endif  // CONTROL_MODE_REMAPPER_HPP
