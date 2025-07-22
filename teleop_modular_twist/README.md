# teleop_modular_twist/TwistControlMode

This is a feature-rich ControlMode that sends [Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html) and [TwistStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/TwistStamped.html) messages.

### Inputs

#### Axes
- `linear.x`: The x component of the linear velocity
- `linear.y`: The y component of the linear velocity
- `linear.z`: The z component of the linear velocity
- `angular.x`: The x component of the angular velocity
- `angular.y`: The y component of the angular velocity
- `angular.z`: The z component of the angular velocity


- `speed`: The input axis that scales the output speed from 0 to 1. *(Required if `use_speed_input` is set to true)*

> **Note**: `linear.*` and `angular.*` inputs are multiplied by the `speed` input (when `use_speed_input` is set to 
> true) and the `scale.linear.*` / `scale.angular.*` parameters in the output twist message. 
> 
> If you don't want to use an additional `speed` input, you can disable it by setting `use_speed_input` to false in 
> your parameter file. It is disabled by default.
 
> **Note**: `linear.*` and `angular.*` are usually values between -1 and 1 that you then turn into a velocity unit by 
> setting `scale.linear.*` and `scale.angular.*` parameters to be your largest desired speed. 
> 
> However, if you have some novel input device that provides inputs in metres per second, you can keep the scale at 1 
> and it should still work!

#### Buttons

- `locked`: When true, the control mode will send all zeroes to make the robot halt. *(optional)*

### Parameters

Parameters are defined in [twist_control_mode_parameters.yaml](./src/twist_control_mode_parameters.yaml). 

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

    # Topic to send Twist messages to. (Optional if stamped_topic is set)
    topic: "/turtle1/cmd_vel"

    # Topic to send TwistStamped messages to. (Optional if topic is set) 
    stamped_topic: "/cmd_vel_stamped"

    # Use the input named "speed" to scale everything
    use_speed_input: true

    # Used as the header.frame_id of all sent TwistStamped messages. 
    # The name of the reference frame twist messages are relative to. (Optional)
    frame: "endeffector"
    
    # Values to multiply inputs with. All values are 1.0 by default. (Recommended)
    scale:
      linear:
        x: 0.5
        y: 0.5
        z: 0.35
        # Or if you dont want to specify per-component, you can just set all
        all: 1.0
      angular:
        x: 0.5
        y: 0.5
        z: 0.35
        # Or if you dont want to specify per-component, you can just set all
        all: 1.0

    # Prevent velocities larger than the specified values from being emitted
    # If you leave a parameter unspecified, no limits will be applied. (Optional)
    limits: 
      linear:
        x: 1.0
        y: 1.0
        z: 0.5
        # Or you can provide a default by setting 'all'. This has no effect if you 
        # set values for each component. If not set, any unspecified x,y,z will not 
        # be limited.
        all: 1.0
        # False by default. When true, the magnitude of the velocity will be 
        # limited rather than individual axes. Limiting will still be done  
        # proportionally to the provided x,y,z limits, which don't have to be
        # uniform.
        normalized: true
        # False by default. When true, limits are scaled with the 'speed' input.
        scale_with_speed: true
      angular:
        x: 1.0
        y: 1.0
        z: 0.5
        # Or you can provide a default by setting 'all'. This has no effect if you 
        # set values for each component. If not set, any unspecified x,y,z will not 
        # be limited.
        all: 1.0
        # False by default. When true, the magnitude of the velocity will be 
        # limited rather than individual axes. Limiting will still be done  
        # proportionally to the provided x,y,z limits, which don't have to be
        # uniform.
        normalized: true
        # False by default. When true, limits are scaled with the 'speed' input.
        scale_with_speed: true
      
```
