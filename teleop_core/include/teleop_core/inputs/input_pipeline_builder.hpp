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
     *
     */
    virtual void link_inputs(const InputManager::Props& previous, InputManager::Props& next, const std::set<std::string>& declared_names) = 0;

    /**
     * Allows an element to declare that it wants some
     */
    virtual void declare_input_names(std::set<std::string>& names);
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
    }

    previously_linked_ = true;
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
    elements_.emplace_back(element, InputManager::Props{}, [index, this](){
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
  InputPipelineBuilder() = default;

  /**
   * Creates a new pipeline from a vector of pipeline elements
   */
  InputPipelineBuilder(const std::vector<std::reference_wrapper<Element>>& elements) {
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
      : element(element), next(next), relink_callback(callback) {}
  };

  std::vector<ElementHandle> elements_{};
  std::set<std::string> declared_names_{};

  bool previously_linked_ = false;
  bool previously_declared_names_ = false;

  // TODO: Make construct, and add mechanism to trigger partial rebuild
};

}  // namespace teleop


#endif  // CONTROL_MODE_INPUTPIPELINEBUILDER_HPP
