//
// Created by nova on 7/2/25.
//

#include "teleop/inputs/InputCollection.hpp"

namespace teleop {

template<>
void InputCollection<Button>::setup_new_item(const std::shared_ptr<Button> &item) {
  item->export_events(events_);
}

template<>
void InputCollection<Axis>::setup_new_item(const std::shared_ptr<Axis> &item) {
  item->export_events(events_);
}

} // namespace teleop
