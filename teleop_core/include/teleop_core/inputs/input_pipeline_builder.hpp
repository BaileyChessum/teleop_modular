//
// Created by Bailey Chessum on 29/8/25.
//

#ifndef CONTROL_MODE_INPUTPIPELINEBUILDER_HPP
#define CONTROL_MODE_INPUTPIPELINEBUILDER_HPP

#include "teleop_core/inputs/InputMapBuilder.hpp"
#include "teleop_core/inputs/InputManager.hpp"
#include <vector>

namespace teleop {

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
    /**
     * Add inputs to the builder.
     * \param[in] previous The result of the previous InputPipelineBuilder::Element, to use as a basis for populating
     * next.
     * \param[in,out] next The result of this Element. Always stores the previous result from this Element.
     *
     */
    virtual void link_inputs(const InputManager::Props& previous, InputManager::Props& next) = 0;
  };

  InputPipelineBuilder(const std::vector<std::reference_wrapper<Element>>& elements) {
    this->elements.reserve(elements.size());
    for (auto& element : elements)
      this->elements.emplace_back(element, InputManager::Props{});
  }

private:
  /**
   * Holds a InputManager::Props alongside the Element that populates it
   *
   */
  struct ElementHandle {
    std::reference_wrapper<Element> element;
    InputManager::Props next;

    ElementHandle(std::reference_wrapper<Element> element, InputManager::Props next) : element(element), next(next) {}
  };

  std::vector<ElementHandle> elements{};

  // TODO: Make construct, and add mechanism to trigger partial rebuild
};

}  // namespace teleop


#endif  // CONTROL_MODE_INPUTPIPELINEBUILDER_HPP
