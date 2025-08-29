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
  class Element {

    /**
     * Add inputs to the builder.
     */
    virtual void link_inputs(const InputManager::Props& previous, InputManager::Props& next) = 0;

  };

  InputPipelineBuilder(const std::vector<std::reference_wrapper<Element>>& elements) {
    for (auto& element : elements) {

      this->elements.emplace_back(element, InputManager::Props{});
    }
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
