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

#include "teleop_core/events/EventCollection.hpp"
#include "teleop_core/events/ButtonEvent.hpp"

namespace teleop
{

EventCollection::EventCollection(
  std::weak_ptr<control_mode::internal::EventListenerQueue> listener_queue,
  InputManager & inputs)
: listener_queue_(std::move(listener_queue))
  , factory_{{"/down",
      [](const std::string & name, const std::weak_ptr<control_mode::internal::EventListenerQueue> & queue, EventCollection & self) {
        auto event = std::make_shared<internal::ButtonEvent>(name, queue, true);
        self.add_child_element(event);
        return event;
      }},
    {"/up",
      [](const std::string & name, const std::weak_ptr<control_mode::internal::EventListenerQueue> & queue, EventCollection& self) {
        auto event = std::make_shared<internal::ButtonEvent>(name, queue, false);
        self.add_child_element(event);
        return event;
      }}}
{
}

void EventCollection::clean_up()
{
  for (auto it = items_.begin(); it != items_.end(); ) {
    if (it->second.expired()) {
      it = items_.erase(it);
    } else {
      ++it;
    }
  }
}

size_t EventCollection::size() const
{
  size_t count = 0;
  for (const auto & item : items_) {
    if (!item.second.expired()) {
      ++count;
    }
  }
  return count;
}

control_mode::Event::SharedPtr EventCollection::operator[](const std::string & index)
{
  auto it = items_.find(index);
  Event::SharedPtr ptr;

  if (it != items_.end()) {
    ptr = it->second.lock();
    if (!ptr) {
      items_.erase(it);
    }
  }

  if (!ptr) {
    ptr = create_event_for_name(index);
    items_[index] = ptr;
  }

  return ptr;
}

/// Used with create_event_for_name
bool EventCollection::ends_with(const std::string & str, const std::string & suffix)
{
  return str.size() >= suffix.size() && str.compare(
    str.size() - suffix.size(), suffix.size(),
    suffix) == 0;
}

control_mode::Event::SharedPtr EventCollection::create_event_for_name(const std::string & name)
{
  RCLCPP_DEBUG(logger_, "Creating event for name %s", name.c_str());

  // Check if any special suffixes are included
  auto it =
    std::find_if(
    factory_.begin(), factory_.end(), [&](const auto & item) {
      return ends_with(name, item.first);
    });

  if (it != factory_.end()) {
    return it->second(name, listener_queue_, *this);
  }

  // Otherwise create a standard Event that can be invoked with .invoke() externally
  return std::make_shared<Event>(name, listener_queue_);
}

void EventCollection::clear_invalid_child_elements()
{
  child_elements_.erase(
      std::remove_if(
          child_elements_.begin(),
          child_elements_.end(),
          [](const std::weak_ptr<InputPipelineBuilder::Element>& ptr) { return ptr.expired(); }
      ),
      child_elements_.end()
  );

}
void EventCollection::add_child_element(const std::shared_ptr<InputPipelineBuilder::Element>& element)
{
  child_elements_.emplace_back(element);
  if (pipeline_previously_linked_) {
    // TODO: Make it possible to dynamically register button events, propegating input names down
    RCLCPP_ERROR(logger_, "Dynamic event registration into the pipeline is yet to be implemented! "
                 "Your button names will not be properly declared to any remapping parameters.");
    relink_pipeline();
  }
}

void EventCollection::on_inputs_available(InputManager::Hardened& inputs)
{
  clear_invalid_child_elements();

  for (const auto& weak_ptr : child_elements_) {
    if (auto child = weak_ptr.lock())
      child->on_inputs_available(inputs);
  }

  pipeline_previously_linked_ = true;
}

void EventCollection::relink()
{
  relink_pipeline();
}

void EventCollection::declare_input_names(InputPipelineBuilder::DeclaredNames& names)
{
  clear_invalid_child_elements();

  for (const auto& weak_ptr : child_elements_) {
    if (auto child = weak_ptr.lock())
      child->declare_input_names(names);
  }
}

}  // namespace teleop
