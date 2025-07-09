//
// Created by Bailey Chessum on 11/6/25
//

#ifndef TELEOP_MODULAR_INPUTSOURCEUPDATEDELEGATE_HPP
#define TELEOP_MODULAR_INPUTSOURCEUPDATEDELEGATE_HPP
#include <rclcpp/time.hpp>

namespace teleop_modular
{

/**
 * An abstract base class which exposes a method allowing InputSources to notify its owner that it has received new
 * data. The owner can then use this to invoke a new update.
 */
class InputSourceUpdateDelegate
{
public:
  virtual ~InputSourceUpdateDelegate() = default;

  /**
   * Alerts the owner of the calling InputSource that a new input has been received, and to request an update.
   * @param now The time to associate with the input the update is being requested for.
   */
  virtual void on_input_source_requested_update(const rclcpp::Time& now) = 0;
};

}  // namespace teleop_modular

#endif  // TELEOP_MODULAR_INPUTSOURCEUPDATEDELEGATE_HPP
