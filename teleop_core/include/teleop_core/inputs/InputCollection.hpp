// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Created by nova on 7/1/25.
//

#ifndef TELEOP_MODULAR_INPUTCOLLECTION_HPP
#define TELEOP_MODULAR_INPUTCOLLECTION_HPP

#include "teleop_core/inputs/Button.hpp"
#include "teleop_core/inputs/Axis.hpp"
#include "InputCommon.hpp"
#include "teleop_core/utilities/WeakMapIterator.hpp"
#include <functional>
#include <map>
#include "control_mode/input_collection.hpp"

namespace teleop
{

template<typename InputT>
class InputCollection
{
public:
  explicit InputCollection() = default;

  // Add move constructor
  InputCollection(InputCollection && other) noexcept
  : items_(std::move(other.items_))
  {
  }

  // Add move assignment
  InputCollection & operator=(InputCollection && other) noexcept
  {
    if (this != &other) {
      items_ = std::move(other.items_);
    }
    return *this;
  }

  // Delete copy constructor and assignment
  InputCollection(const InputCollection &) = delete;
  InputCollection & operator=(const InputCollection &) = delete;

  // Container type aliases
  using iterator = utils::WeakMapIterator<InputT, false>;
  using const_iterator = utils::WeakMapIterator<InputT, true>;

  std::shared_ptr<InputT> operator[](const std::string & name)
  {
    // Find the element
    auto it = items_.find(name);
    std::shared_ptr<InputT> ptr;

    if (it != items_.end()) {
      ptr = it->second.lock();
      if (!ptr) {
        // If the weak_ptr expired, remove it from the map
        items_.erase(it);
      }
    }

    if (!ptr) {
      // Create new input if we don't have a valid one
      ptr = std::make_shared<InputT>(name);
      setup_new_item(ptr);
      items_[name] = ptr;
    }

    return ptr;
  }

  iterator begin()
  {
    return iterator(items_.begin(), &items_);
  }
  iterator end()
  {
    return iterator(items_.end(), &items_);
  }
  [[nodiscard]] const_iterator begin() const
  {
    return const_iterator(items_.begin(), &items_);
  }
  [[nodiscard]] const_iterator end() const
  {
    return const_iterator(items_.end(), &items_);
  }
  [[nodiscard]] const_iterator cbegin() const
  {
    return const_iterator(items_.begin(), &items_);
  }
  [[nodiscard]] const_iterator cend() const
  {
    return const_iterator(items_.end(), &items_);
  }

  /**
   * Utility method to delete expired weak pointers from the internal map
   */
  void clean_up()
  {
    for (auto it = items_.begin(); it != items_.end(); ) {
      if (it->second.expired()) {
        it = items_.erase(it);
      } else {
        ++it;
      }
    }
  }

  /**
   * Gets the number of non-expired inputs
   */
  [[nodiscard]] size_t size() const
  {
    size_t count = 0;
    for (const auto & item : items_) {
      if (!item.second.expired()) {
        ++count;
      }
    }
    return count;
  }

  class ControlModeCompat : public control_mode::InputCollection<typename InputT::ControlModeType>
  {
public:
    ControlModeCompat(InputCollection & parent)
    : parent_(parent)
    {
    }

    typename InputT::ControlModeType::SharedPtr operator[](const std::string & name) override
    {
      return parent_[name];
    }

private:
    InputCollection & parent_;
  };

  ControlModeCompat get_control_mode_compat()
  {
    return ControlModeCompat(*this);
  }

private:
  void setup_new_item(const std::shared_ptr<InputT> & item);
  std::map<std::string, std::weak_ptr<InputT>> items_{};
  //  std::reference_wrapper<EventCollection> events_;
};

template<>
void InputCollection<Button>::setup_new_item(const std::shared_ptr<Button> & item);

template<>
void InputCollection<Axis>::setup_new_item(const std::shared_ptr<Axis> & item);

}  // namespace teleop

#endif  // TELEOP_MODULAR_INPUTCOLLECTION_HPP
