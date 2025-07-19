//
// Created by Bailey Chessum on 11/6/25
//

#ifndef TELEOP_MODULAR_INPUT_SOURCE_UPDATE_DELEGATE_HPP
#define TELEOP_MODULAR_INPUT_SOURCE_UPDATE_DELEGATE_HPP

#include "visibility_control.h"
#include <rclcpp/time.hpp>

namespace input_source
{

/**
 * @class An abstract base class which exposes a method allowing InputSources to notify its owner that it has received
 * new data. The owner can then use this to invoke a new update.
 */
class INPUT_SOURCE_PUBLIC UpdateDelegate
{
public:
  virtual ~UpdateDelegate() = default;

  /**
   * Alerts the owner of the calling InputSource that a new input has been received, and to request an update.
   * @param now The time to associate with the input the update is being requested for.
   */
  virtual void on_input_source_requested_update(const rclcpp::Time & now) = 0;
};

}  // namespace input_source

#endif  // TELEOP_MODULAR_INPUT_SOURCE_UPDATE_DELEGATE_HPP
