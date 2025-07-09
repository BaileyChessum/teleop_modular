//
// Created by nova on 7/2/25.
//

#include "teleop/inputs/InputCollection.hpp"

namespace teleop
{

// TODO: Make these redundant
template <>
void InputCollection<Button>::setup_new_item(const std::shared_ptr<Button>& item)
{
}

template <>
void InputCollection<Axis>::setup_new_item(const std::shared_ptr<Axis>& item)
{
}

}  // namespace teleop
