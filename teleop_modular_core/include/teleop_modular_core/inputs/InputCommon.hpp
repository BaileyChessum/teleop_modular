//
// Created by Bailey Chessum on 4/6/25.
//

#ifndef TELEOP_MODULAR_INPUTCOMMON_HPP
#define TELEOP_MODULAR_INPUTCOMMON_HPP

#include <rclcpp/time.hpp>
#include <utility>

#include "teleop_modular_core/inputs/state/InputDeclaration.hpp"
#include "control_mode/input_interface.hpp"

namespace teleop
{

template <typename T>
class InputCommon : public control_mode::InputInterface<T>
{
public:
  virtual ~InputCommon() = default;

  T value() override;

  void debounce(const rclcpp::Time& now);

  /**
   * @returns true if the value changed since last debounce
   */
  [[nodiscard]] bool changed() const;

  /**
   * Adds the given definition to the input
   */
  void add_definition(const std::reference_wrapper<T>& definition)
  {
    definitions_.emplace_back(definition);
  }

  /**
   * Removes the first instance of the given definition
   */
  void remove_definition(const std::reference_wrapper<T>& definition)
  {
    using Ref = std::reference_wrapper<T>;
    auto same_object = [&](const Ref& ref) { return std::addressof(ref.get()) == std::addressof(definition.get()); };

    // Erases all instances
    // definitions_.erase(
    //   std::remove_if(definitions_.begin(), definitions_.end(), same_object),
    //   definitions_.end());

    auto it = std::find_if(definitions_.begin(), definitions_.end(), same_object);
    if (it != definitions_.end())
      definitions_.erase(it);
  }

protected:
  // Constructor is protected, as to ensure plugin classes don't accidentally use this type directly
  explicit InputCommon(std::string name) : control_mode::InputInterface<T>(std::move(name))
  {
  }

private:
  // Consolidates multiple definitions into a single value
  inline T accumulate_value()
  {
    if (definitions_.empty())
      return params_.default_value;

    auto it = definitions_.begin();
    T accumulator = *it;
    it++;

    while (it != definitions_.end())
    {
      accumulator += *it;
      it++;
    }

    return accumulator;
  }

  struct Params
  {
    T default_value = 0;
  };

  Params params_;

  T previous_debounce_value_ = 0;
  T current_debounce_value_ = 0;

  /// Reference wrappers obtained through InputDeclaration<T>s, that provide the value for the input
  std::vector<std::reference_wrapper<T>> definitions_{};
};

template <typename T>
bool InputCommon<T>::changed() const
{
  return previous_debounce_value_ != current_debounce_value_;
}

}  // namespace teleop

#endif  // TELEOP_MODULAR_INPUTCOMMON_HPP
