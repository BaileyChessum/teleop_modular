// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
//
// Created by nova on 6/29/25.
//

#ifndef TELEOP_MODULAR_EVENTCOLLECTION_H
#define TELEOP_MODULAR_EVENTCOLLECTION_H

#include <utility>
#include <vector>
#include <memory>
#include <map>
#include "teleop_core/events/Event.hpp"
#include "teleop_core/utilities/WeakMapIterator.hpp"
#include "teleop_core/inputs/InputManager.hpp"
#include "control_mode/event/event_collection.hpp"
#include "teleop_core/inputs/input_pipeline_builder.hpp"

namespace teleop
{

/**
 * A container of Events, where events that don't yet exist are created when an attempt is made to retrieve them.
 */
class EventCollection : public control_mode::EventCollection, public InputPipelineBuilder::Element,
                        public InputPipelineElementDelegate
{
  using Event = control_mode::Event;
public:
  explicit EventCollection(
    std::weak_ptr<control_mode::internal::EventListenerQueue> listener_queue,
    InputManager & inputs);

  using iterator = utils::WeakMapIterator<Event, false>;
  using const_iterator = utils::WeakMapIterator<Event, true>;

  control_mode::Event::SharedPtr operator[](const std::string & index) override;

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

  void clean_up();

  /**
   * Gets the number of non-expired inputs
   */
  [[nodiscard]] size_t size() const;

  static bool ends_with(const std::string & str, const std::string & suffix);

  void link_inputs(const InputManager::Props& previous, InputManager::Props& next, const DeclaredNames& names) override {};

  // TODO: Rename to make clear that these are the inputs we want to consume, not provide
  /**
   * Allows an element to declare what inputs it CONSUMES, not provides. This is useful for any dynamic remapping of
   * any previous elements in the pipeline.
   * \param[in, out] names the set accumulating all declared input names. Add names to declare to this set.
   */
  void declare_input_names(DeclaredNames& names) override;;

  /**
     * Callback ran when hardened inputs are available.
   */
  void on_inputs_available(InputManager::Hardened& inputs) override;;

  void relink() override;

private:
  void add_child_element(const std::shared_ptr<InputPipelineBuilder::Element>& element);

  void clear_invalid_child_elements();

  /**
   * Parses the given event name in creating an event, and creates the appropriate event based on that name. For
   * example, events ending in "/down" or "/up" will be events for buttons being pressed down or released.
   * @param[in] name    The name of the event to create. This will also inform the type of event created.
   */
  control_mode::Event::SharedPtr create_event_for_name(const std::string & name);

  std::map<std::string, std::weak_ptr<Event>> items_{};
  std::weak_ptr<control_mode::internal::EventListenerQueue> listener_queue_;

  using FactoryFunc = std::function<std::shared_ptr<Event>(
      const std::string &,
      const std::weak_ptr<control_mode::internal::EventListenerQueue> &,
      EventCollection &)>;

  /// Used to create special subclasses of Event for various different suffixes
  const std::unordered_map<std::string, FactoryFunc> factory_;
  std::vector<std::weak_ptr<InputPipelineBuilder::Element>> child_elements_;

  rclcpp::Logger logger_ = rclcpp::get_logger("event_collection");
  bool pipeline_previously_linked_ = false;
};

}  // namespace teleop

#endif  // TELEOP_MODULAR_EVENTCOLLECTION_H
