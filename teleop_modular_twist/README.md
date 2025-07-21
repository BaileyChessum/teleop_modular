# teleop_modular_twist/TwistControlMode

This is a ControlMode that sends [Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html) and [TwistStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/TwistStamped.html) messages.

### Inputs

#### Axes
- `speed`: The input axis that scales the output speed from 0 to 1.
- `twist_x`: The x component of the linear velocity from -1 to 1.
- `twist_y`: The y component of the linear velocity from -1 to 1.
- `twist_z`: The z component of the linear velocity from -1 to 1.
- `twist_roll `: The x component of the angular velocity from -1 to 1.
- `twist_pitch`: The y component of the angular velocity from -1 to 1.
- `twist_yaw  `: The z component of the angular velocity from -1 to 1.

> **Note**: `twist_*` inputs are multiplied by `speed` and the `max_speed.linear`/`max_speed.angular` parameter in the output twist message

#### Buttons

- `locked`: When true, the control mode will send all zeroes to make the robot halt. *(optional)*

### Parameters

Parameters are defined in [twist_control_mode_parameters.yaml](./src/twist_control_mode_parameters.yaml). Please refer to the file for a full list of parameters.

```yaml
# Example parameter file
teleop_node:
  ros__parameters:
    control_modes:
      names: [ "twist_control_mode" ]
      twist_control_mode:
        type: "teleop_modular_twist/TwistControlMode"

twist_control_mode:
  ros__parameters:

    # Topic to send Twist messages to. (Required)
    topic: "/turtle1/cmd_vel"

    # Used as the header.frame_id of all sent TwistStamped messages. 
    # The name of the reference frame twist messages are relative to. (Optional)
    frame_id: "endeffector"

    max_speed:
      # The maximum value for the linear component of the twist messages, in metres per second. 
      # A 'twist_*' of 1 and 'speed' of 1 will correspond to this value being sent. (Required)
      linear: 0.25
      # The maximum value for the angular component of the twist messages, in radians per second.
      # A 'twist_*' of 1 and 'speed' of 1 will correspond to this value being sent. (Required)
      angular: 0.25
      # When true, the magnitude of the velocity will be limited to be less than or equal to the max speed, 
      # rather than individual axes. (Optional, defaults to true)
      normalized: true
```
