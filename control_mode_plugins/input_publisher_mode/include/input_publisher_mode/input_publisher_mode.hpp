#ifndef INPUT_PUBLISHER_MODE__INPUT_PUBLISHER_MODE_HPP_
#define INPUT_PUBLISHER_MODE__INPUT_PUBLISHER_MODE_HPP_

#include <rclcpp/time.hpp>
#include <string>
#include "control_mode/control_mode.hpp"
#include "input_publisher_mode/visibility_control.h"

namespace input_publisher_mode
{

using namespace control_mode;

/**
 * \class A generic control mode that forwards inputs from teleop_modular.
 */
class INPUT_PUBLISHER_MODE_PUBLIC InputPublisherMode : public ControlMode
{
public:
  InputPublisherMode();

  return_type on_init() override;
  CallbackReturn on_configure(const State & previous_state) override;
  void on_capture_inputs(Inputs inputs) override;

  /**
   * \brief Publishes a message to tell the control system to do nothing. Used when the control mode is locked, and
   * called once when deactivated.
   *
   * \param[in] now The time to associate with the 'halt' message.
   */
  void publish_halt_message(const rclcpp::Time & now) const;

  CallbackReturn on_activate(const State & previous_state) override;
  return_type on_update(const rclcpp::Time & now, const rclcpp::Duration & period) override;

  CallbackReturn on_deactivate(const State & previous_state) override;
  CallbackReturn on_cleanup(const State & previous_state) override;
  CallbackReturn on_error(const State & previous_state) override;
  CallbackReturn on_shutdown(const State & previous_state) override;

protected:
  ~InputPublisherMode() override;

private:
  /// Helper struct to hold parameters used by the control mode.
  struct Params {
    /// The topic name to send messages to.
    std::string topic = "";
    /// The ROS2 topic Quality of Service value to use in publisher_.
    int qos = 10;
  };

  /// Stores current parameter values
  Params params_;

  // TODO: Set an appropriate message type for the publisher, then uncomment its declaration/usages
  // rclcpp::Publisher<TODO>::SharedPtr publisher_;

  // TODO: Add shared pointers for any buttons/axes you need here, then set them in on_capture_inputs().

  // You can hold references to inputs like this, and set their values in on_capture_inputs:
  /// Input from 0 to 1 that directly scales the output speed.
  Axis::SharedPtr speed_;

  /// If you need to store some unknown number N inputs and associated params, you can make a helper struct like this,
  /// then store an array of them. If you had one input per joint, this could be 'JointHandle' for example, with an
  /// `std::vector<JointHandle> joints_;` to hold them all.
  // struct InputHandle {
  //   Axis::SharedPtr axis_;  //< Set this in on_capture_inputs
  //   float some_param_;      //< Set this in on_configure
  // };
};

}  // namespace input_publisher_mode

#endif  // INPUT_PUBLISHER_MODE__INPUT_PUBLISHER_MODE_HPP_
