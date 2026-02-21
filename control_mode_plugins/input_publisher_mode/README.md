# input_publisher_mode/InputPublisherMode

A generic control mode that publishes inputs from teleop_modular.

Publishes [teleop_msgs/InputNames](../../teleop_msgs/msg/InputNames.msg) and [teleop_msgs/CombinedInputValues](../../teleop_msgs/msg/CombinedInputValues.msg) messages.

### Inputs

#### Axes

Publishes all Axes as configured in the configuration file.

#### Buttons

Publishes all Buttons as configured in the configuration file.

- `locked`: When true, the control mode will send all zeroes to make the robot halt. *(optional)*

> **Note**: `locked` is automatically captured by teleop_modular and exposed through the `bool is_locked()` method.

### Parameters

- `input_names_topic : string` The topic name to send input name messages to (Required)
- `inputs_topic : string` The topic name to send input messages to (Optional, defaults to input_names_topic + "/values")
- `inputs_qos : int` The ROS2 topic Quality of Service value to use in the inputs publisher (Optional, defaults to 10) 
- `axis_names : string[]` The list of Axis names to publish. (One of axis_names or button_names required)
- `button_names : string[]` The list of Button names to publish. (One of axis_names or button_names required)

```yaml
# Example parameter file
teleop_node:
  ros__parameters:
    control_modes:
      names: [ "input_publisher_mode" ]
      input_publisher_mode:
        type: "input_publisher_mode/InputPublisherMode"

input_publisher_mode:
  ros__parameters:
    # Topic to send name messages to (Required)
    input_names_topic: "/turtle1/input"

    # Topic to send input messages to. (Optional, defaults to input_names_topic + "/values")
    inputs_topic: "/turtle1/input/values";
    
    # The ROS2 topic Quality of Service value to use in the inputs publisher (Optional, defaults to 10)
    inputs_qos: 10

    # List of Axis names to publish. (One of axis_names or button_names required)
    axis_names:
      - "x_effort"
      - "y_effort"
    
    # List of Button names to publish. (One of axis_names or button_names required)
    button_names:
      - "fast_mode"
```
