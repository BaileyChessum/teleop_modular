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
#include <utility>
#include <vector>
#include <set>
#include <functional>
#include <optional>
#include <atomic>

namespace teleop {

class InputPipelineElementDelegate {
public:
  virtual void relink() = 0;

  /**
   * Requests a full pipeline rebuild from element 0, including re-declaring all input names.
   * Unlike relink(), this is deferred and executed safely on the update thread via
   * InputPipelineBuilder::flush_pending_relink(). Elements that hold a pipeline delegate can use
   * this to schedule a rebuild without risking concurrent modification of the InputMap.
   *
   * Default implementation falls back to relink() for delegates that don't support deferred rebuilds.
   */
  virtual void request_full_relink() { relink(); }
};

class PositionalInputPipelineElementDelegate : public InputPipelineElementDelegate {
public:
  virtual void relink_from(size_t index) = 0;
  void relink() override {
    relink_from(0);
  }
};

/**
 * Manages multiple things that add stuff to InputMapBuilders for InputManager::Props, but which might need to retrigger
 * the rest of the build process down the line.
 */
class InputPipelineBuilder : public PositionalInputPipelineElementDelegate
{
public:
  /**
   * Stores a set of names for button and axis
   */
  struct DeclaredNames {
    std::set<std::string> axis_names{};
    std::set<std::string> button_names{};
  };

  /**
   * Something that contributes to building the input pipeline, such as input sources.
   */
  class Element {
  public:
    using DeclaredNames = InputPipelineBuilder::DeclaredNames;

    /**
     * Add inputs to the builder.
     * \param[in] previous The result of the previous InputPipelineBuilder::Element, to use as a basis for populating
     * next.
     * \param[in,out] next The result of this Element. Always stores the previous result from this Element.
     */
    virtual void link_inputs(const InputManager::Props& previous, InputManager::Props& next, const DeclaredNames& names) = 0;

    // TODO: Rename to make clear that these are the inputs we want to consume, not provide
    /**
     * Allows an element to declare what inputs it CONSUMES, not provides. This is useful for any dynamic remapping of
     * any previous elements in the pipeline.
     * \param[in, out] names the set accumulating all declared input names. Add names to declare to this set.
     */
    virtual void declare_input_names(DeclaredNames& names) {};

    /**
     * Callback ran when hardened inputs are available.
     */
    virtual void on_inputs_available(InputManager::Hardened& inputs) = 0;

    /**
     * Internal method used to set up callbacks for when you call relink_pipeline()
     */
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

      delegate_.value()->relink();
    }

    /**
     * Requests a full pipeline rebuild from element 0, including re-declaring all input names.
     * This is safe to call from any thread — the actual rebuild is deferred and runs on the update
     * thread via InputPipelineBuilder::flush_pending_relink(). Use this instead of relink_pipeline()
     * when a new element has been added to the pipeline after initial construction.
     */
    void request_full_pipeline_relink() {
      if (!delegate_.has_value()) {
        RCLCPP_ERROR(rclcpp::get_logger("input_pipeline_builder"),
                     "InputPipelineBuilder::Element tried to request_full_pipeline_relink(), but the delegate used to "
                     "do so was never set.");
        return;
      }

      delegate_.value()->request_full_relink();
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
  void relink_from(size_t index) override {
    auto logger = rclcpp::get_logger("input_pipeline_builder");
    RCLCPP_DEBUG(logger, "Relinking input pipline from element %lu", index);

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

      RCLCPP_DEBUG(logger, "Relinking input pipline element %lu", i);
      element.element.get().link_inputs(previous_inputs, element.next, names_);
      previous_inputs = element.next;
    }

    previously_linked_ = true;

    // Harden the inputs
    RCLCPP_DEBUG(logger, "Hardening inputs");
    auto hardened = target_.init(elements_[elements_.size() - 1].next);

    for (size_t i = index; i < elements_.size(); i++) {
      RCLCPP_DEBUG(logger, "Providing inputs to input pipeline element %lu", i);
      elements_[i].element.get().on_inputs_available(hardened);
    }
  }

  /**
   * Declares input names requested by any input consumers.
   */
  void declare_names() {
    // Iterate backwards over elements_ calling declare_input_names
    names_ = DeclaredNames();
    for (size_t i = elements_.size(); i > 0; i--) {
      elements_[i - 1].element.get().declare_input_names(names_);
    }
    previously_declared_names_ = true;
  }

  /**
   * Schedules a full pipeline rebuild — re-declares all input names then re-links and hardens
   * from element 0. Thread-safe: can be called from any thread. The rebuild itself runs on the
   * update thread when flush_pending_relink() is next called.
   */
  void request_full_relink() override {
    full_relink_pending_.store(true, std::memory_order_release);
  }

  /**
   * If a full relink was requested (via request_full_relink()), performs it now. Call this at
   * the top of each update cycle on the update thread, before any reads from the InputMap.
   */
  void flush_pending_relink() {
    if (full_relink_pending_.exchange(false, std::memory_order_acq_rel)) {
      previously_declared_names_ = false;
      relink_from(0);
    }
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

    elements_.emplace_back(element, previous_inputs, *this, index);

    // Update the pipeline if previously set up
    if (previously_declared_names_)
      declare_names();
    if (previously_linked_)
      relink_from(index);
  }

  /**
   * Creates a new pipeline without any elements
   */
  explicit InputPipelineBuilder(InputManager& target) : target_(target) {}

  /**
   * Used for unit tests. Removes all elements. Does not relink after clearing.
   */
  void clear() {
    elements_.clear();
    names_ = DeclaredNames();
    previously_linked_ = false;
    previously_declared_names_ = false;
    full_relink_pending_.store(false, std::memory_order_relaxed);
    target_.init(InputManager::Props());
  }

  /**
   * Creates a new pipeline from a vector of pipeline elements
   */
  InputPipelineBuilder(const std::vector<std::reference_wrapper<Element>>& elements, InputManager& target) : target_(target) {
    elements_.reserve(elements.size());
    for (size_t i = 0; i < elements.size(); i++) {
      auto& element = elements[i];
      elements_.emplace_back(element, InputManager::Props{}, *this, i);
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

    PositionalInputPipelineElementDelegate & delegate;
    size_t position;

    void relink() override {
      auto logger = rclcpp::get_logger("input_pipeline_element_handle");
      RCLCPP_DEBUG(logger, "Relinking inputs.");

      delegate.relink_from(position);
    }

    void request_full_relink() override {
      delegate.request_full_relink();
    }

    ElementHandle(
        std::reference_wrapper<Element> element,
        InputManager::Props next,
        PositionalInputPipelineElementDelegate & delegate,
        size_t position)
      : element(element), next(std::move(next)), delegate(delegate), position(position)
    {
      element.get().set_pipeline_delegate(*this);
    }
  };

  InputManager& target_;

  std::vector<ElementHandle> elements_{};
  DeclaredNames names_{};

  bool previously_linked_ = false;
  bool previously_declared_names_ = false;
  std::atomic<bool> full_relink_pending_{false};
};

}  // namespace teleop


#endif  // CONTROL_MODE_INPUTPIPELINEBUILDER_HPP
