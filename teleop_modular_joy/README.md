# teleop_modular_joy

An `InputSource` plugin for `teleop_modular` that reads joystick input from `sensor_msgs/Joy` messages.

## Overview

`teleop_modular_joy` subscribes to a Joy topic and exposes the joystick's axes and buttons as named inputs that can be remapped and consumed by any `ControlMode` plugin.

Plugin type: `teleop_modular_joy/JoyInputSource`

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `topic` | `string` | `/joy` | The topic to subscribe to for `sensor_msgs/Joy` messages |
| `axis_definitions` | `string[]` | `[]` | Names assigned to each axis index in the Joy message |
| `button_definitions` | `string[]` | `[]` | Names assigned to each button index in the Joy message |

## Configuration

Axes and buttons are mapped by index -- the first entry in `axis_definitions` corresponds to `axes[0]` in the Joy message, and so on for buttons.

```yaml
joy_input_source:
  ros__parameters:
    topic: "/joy"
    axis_definitions: [
      "left_stick_x",
      "left_stick_y",
      "right_stick_x",
      "right_stick_y",
      "right_trigger"
    ]
    button_definitions: [
      "lock",
      "unlock"
    ]
```

Once names are defined, you can remap and transform them using the standard `remap:` parameters provided by `teleop_modular` to all input sources. See [Input source remapping](https://baileychessum.github.io/teleop_modular/about/input_source_remapping.html) for details.

## See also

- [Writing a teleop package](https://baileychessum.github.io/teleop_modular/guides/writing_a_teleop_package.html)
- [Remapping and transforming inputs](https://baileychessum.github.io/teleop_modular/guides/remapping_and_transforming_inputs.html)
