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
// Created by Bailey Chessum on 01/09/2025.
//

#ifndef EVENT_CORE_FAKE_EVENT_COLLECTION_HPP
#define EVENT_CORE_FAKE_EVENT_COLLECTION_HPP

#include "control_mode/event/event_collection.hpp"
#include "teleop_core/events/EventCollection.hpp"
#include <set>
#include <string>
#include <stdexcept>

namespace teleop::internal
{

/**
 * An input collection that doesn't provide any real values, but collects the names requested by any consumers.
 */
class FakeEventCollection : public control_mode::EventCollection
{
public:
  FakeEventCollection() = default;
  FakeEventCollection(FakeInputCollection<Button>& buttons) : fake_buttons_(&buttons) {}

  /**
   * @brief Gets or constructs an input with the indexed name.
   *
   * The collection may create a new input object if none exists for the given name (or will give you a 'dud' null
   * input, which is still valid to use, but will always be 0).
   *
   * Names of returned inputs may be remapped, such that (*this)[name]->get_name() may not equal name.
   */
  virtual control_mode::Event::SharedPtr operator[](const std::string & name) override {
    names_.insert(name);

    if (!fake_buttons_)
      return default_event();

    if (teleop::EventCollection::ends_with(name, "/down") || teleop::EventCollection::ends_with(name, "/up")) {
      // Get the event name without the /down or /up suffix
      std::size_t pos = name.rfind('/');

      if (pos == std::string::npos)
        throw std::logic_error("Somehow failed to find the / in a string that is guaranteed to have a / in it.");

      // Index with button name to register it
      (*fake_buttons_)[name.substr(0, pos)];
    }

    return default_event();
  }

  /**
   * Gets the set of all input names requested from the collection.
   */
  [[nodiscard]] std::set<std::string>& get_names()
  {
    return names_;
  }

private:
  FakeInputCollection<Button>* fake_buttons_ = nullptr;
  std::set<std::string> names_{};

  static control_mode::Event::SharedPtr default_event() {
    // mutable T shared between all input pointers
    static control_mode::Event::SharedPtr event{std::make_shared<control_mode::Event>("", std::shared_ptr<control_mode::internal::EventListenerQueue>(nullptr))};
    return event;
  }
};

}  // namespace teleop::internal

#endif  // EVENT_CORE_FAKE_EVENT_COLLECTION_HPP
