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
// Created by Bailey Chessum on 29/8/25.
//

#ifndef CONTROL_MODE_INPUTPIPELINEBUILDER_HPP
#define CONTROL_MODE_INPUTPIPELINEBUILDER_HPP

#include "teleop_core/inputs/InputMapBuilder.hpp"
#include "teleop_core/inputs/InputManager.hpp"
#include <vector>
#include <set>
#include <functional>
#include <optional>

namespace teleop {

class InputPipelineElementDelegate {
public:
  virtual void relink() = 0;
};

/**
 * Manages multiple things that add stuff to InputMapBuilders for InputManager::Props, but which might need to retrigger
 * the rest of the build process down the line.
 */
class InputPipelineBuilder
{
public:
  /**
   * Something that contributes to building the input pipeline, such as input sources.
   */
  class Element {
  public:
    /**
     * Add inputs to the builder.
     * \param[in] previous The result of the previous InputPipelineBuilder::Element, to use as a basis for populating
     * next.
     * \param[in,out] next The result of this Element. Always stores the previous result from this Element.
     */
    virtual void link_inputs(const InputManager::Props& previous, InputManager::Props& next, const std::set<std::string>& declared_names) = 0;

    // TODO: Rename to make clear that these are the inputs we want to consume, not provide
    /**
     * Allows an element to declare what inputs it CONSUMES, not provides. This is useful for any dynamic remapping of
     * any previous elements in the pipeline.
     * \param[in, out] names the set accumulating all declared input names. Add names to declare to this set.
     */
    virtual void declare_input_names(std::set<std::string>& names) {};

    /**
     * Callback ran when hardened inputs are available.
     */
    virtual void on_inputs_available(InputManager::Hardened& inputs) = 0;

    void set_pipeline_delegate(InputPipelineElementDelegate& delegate) {
      delegate_ = &delegate;
    }

  protected:
    /**
     * Initiates a new build of the pipeline, calling link_inputs again from this pipeline element onwards.
     * Call this whenever the previous outcome of link_inputs becomes obsolete, and you want to run it again.
     * Should be called as part of the main thread, as it will completely reorganise data used by the main input
     * pipeline.
     */
    void relink_pipeline() {
      if (!delegate_.has_value()) {
        RCLCPP_ERROR(rclcpp::get_logger("input_pipeline_builder"),
                     "InputPipelineBuilder::Element tried to relink_pipeline(), but the delegate used to do so was "
                     "never set.");
        return;
      }

      // TODO: integrate some kind of scheduling/sync mechanism with the main thread to ensure safety. For currest use
      //  cases, we can assume it is already delegated by the main thread as part of event queues

      delegate_.value()->relink();
    }

  private:
    std::optional<InputPipelineElementDelegate*> delegate_ = std::nullopt;
  };

  /**
   * Relinks all inputs from the first element
   */
  void link_inputs() {
    relink_from(0);
  };

  /**
   * Relinks all inputs from elements index and up
   */
  void relink_from(size_t index) {
    if (!previously_declared_names_) {
      declare_names();
    }

    if (!previously_linked_ && index > 0) {
      relink_from(0);
      return;
    }

    auto default_previous_inputs = InputManager::Props();
    auto& previous_inputs = index > 0 ? elements_[index - 1].next : default_previous_inputs;

    for (size_t i = index; i < elements_.size(); i++) {
      auto& element = elements_[i];

      // Setup the initial run
      if (!previously_linked_)
        element.next = previous_inputs;

      element.element.get().link_inputs(previous_inputs, element.next, declared_names_);
      previous_inputs = element.next;
    }

    previously_linked_ = true;

    // Harden the inputs
    auto hardened = target_.init(elements_[elements_.size() - 1].next);

    for (size_t i = index; i < elements_.size(); i++) {
      elements_[i].element.get().on_inputs_available(hardened);
    }
  }

  /**
   * Declares input names requested by any input consumers.
   */
  void declare_names() {
    // Iterate backwards over elements_ calling declare_input_names
    declared_names_.clear();
    for (size_t i = elements_.size(); i > 0; i--) {
      elements_[i - 1].element.get().declare_input_names(declared_names_);
    }
    previously_declared_names_ = true;
  }

  /**
   * Reserves the size of the pipeline
   */
  void reserve(size_t size) {
    elements_.reserve(size);
  }

  /**
   * Adds a new element to the end of the pipeline
   */
  void push_back(Element& element) {
    const auto index = elements_.size();
    auto previous_inputs = index > 0 ? elements_[index - 1].next : InputManager::Props();

    elements_.emplace_back(element, previous_inputs, [index, this](){
      this->relink_from(index);
    });

    // Update the pipeline if previously set up
    if (previously_declared_names_)
      declare_names();
    if (previously_linked_)
      relink_from(index);
  }

  /**
   * Creates a new pipeline without any elements
   */
  InputPipelineBuilder(InputManager& target) : target_(target) {}

  /**
   * Used for unit tests. Removes all elements. Does not relink after clearing.
   */
  void clear() {
    elements_.clear();
    declared_names_.clear();
    previously_linked_ = false;
    previously_declared_names_ = false;
    target_.init(InputManager::Props());
  }

  /**
   * Creates a new pipeline from a vector of pipeline elements
   */
  InputPipelineBuilder(const std::vector<std::reference_wrapper<Element>>& elements, InputManager& target) : target_(target) {
    elements_.reserve(elements.size());
    for (size_t i = 0; i < elements.size(); i++) {
      auto& element = elements[i];
      elements_.emplace_back(element, InputManager::Props{}, [i, this](){
        this->relink_from(i);
      });
    }
  }

private:
  /**
   * Holds a InputManager::Props alongside the Element that populates it
   */
  class ElementHandle : public InputPipelineElementDelegate {
  public:
    std::reference_wrapper<Element> element;
    InputManager::Props next;
    std::function<void()> relink_callback;

    void relink() override {
      relink_callback();
    }

    ElementHandle(std::reference_wrapper<Element> element, InputManager::Props next, std::function<void()> callback)
      : element(element), next(next), relink_callback(callback) {
      element.get().set_pipeline_delegate(*this);
    }
  };

  InputManager& target_;

  std::vector<ElementHandle> elements_{};
  std::set<std::string> declared_names_{};

  bool previously_linked_ = false;
  bool previously_declared_names_ = false;

  // TODO: Make construct, and add mechanism to trigger partial rebuild
};

}  // namespace teleop


#endif  // CONTROL_MODE_INPUTPIPELINEBUILDER_HPP
