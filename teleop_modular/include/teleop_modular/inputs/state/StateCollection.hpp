//
// Created by nova on 7/4/25.
//

#ifndef STATECOLLECTION_HPP
#define STATECOLLECTION_HPP
#include <map>
#include <string>

#include "State.hpp"
#include "teleop_modular/inputs/InputManager.hpp"

namespace teleop_modular
{

/**
 * A collection of stateful input declarations, for use for input values created from things other than an input source.
 * @tparam T The type of value held in the state.
 * @tparam InputT The input type wrapping T (Button for bool, Axis for double).
 */
template <typename T, typename InputT>
class StateCollection
{
public:
  static_assert(std::is_base_of_v<InputCommon<T>, InputT>,
                "InputT must be an input type inheriting from InputCommon (Button, Axis).");

  explicit StateCollection(InputCollection<InputT>& inputs) : inputs_(std::ref(inputs))
  {
  }

  // Delete copy constructor, assignment, and move
  StateCollection(StateCollection&& other) = delete;
  StateCollection& operator=(StateCollection&& other) = delete;
  StateCollection(const StateCollection&) = delete;
  StateCollection& operator=(const StateCollection&) = delete;

  /**
   * @brief Gets the state for a given key, and returns nullptr when there is no item with that key.
   * @param key The name of the input that the state is registered for.
   * @return a shared pointer to the state with the given key as a name if it exists, nullptr otherwise.
   */
  std::shared_ptr<State<T>> operator[](const std::string& key)
  {
    // Find the element
    auto it = items_.find(key);
    std::shared_ptr<State<T>> ptr = nullptr;

    if (it != items_.end())
    {
      ptr = it->second.state;
    }

    return ptr;
  }

  /**
   * Sets the input with a given name to a given value.
   * @param name name The name of the input to define the value for.
   * @param value value The value to define under that input
   */
  void set(const std::string& name, T value)
  {
    if (std::shared_ptr<State<T>> ptr = (*this)[name])
    {
      ptr->value = value;
      return;
    }

    // Make and register a new state
    const auto state = std::make_shared<State<T>>(name, value);
    const auto& input = inputs_.get()[name];

    StateHandle handle{ state, input };

    input->add_definition(handle.state->reference);

    items_.insert({ name, handle });
  }

  /**
   * Remove the input definition with the given name.
   * @param name The name of the input to clear.
   */
  void clear(const std::string& name)
  {
    // Find the element
    auto it = items_.find(name);

    if (it == items_.end())
    {
      // Do nothing, as its already undefined
      return;
    }

    it->second.input->remove_definition(it->second.state->reference);
    items_.erase(name);
  }

private:
  struct StateHandle
  {
    typename State<T>::SharedPtr state;
    /// We need to hold a shared pointer to keep the input alive
    std::shared_ptr<InputT> input;
  };

  std::reference_wrapper<InputCollection<InputT>> inputs_;
  std::map<std::string, StateHandle> items_{};
};

}  // namespace teleop_modular

#endif  // STATECOLLECTION_HPP
