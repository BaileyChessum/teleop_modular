# empty_control_mode/EmptyControlMode

`EmptyControlMode` is a no-op control mode plugin for `teleop_modular`.

It accepts no inputs, publishes nothing, and intentionally performs no work in `on_update()`.

## Usage

```yaml
control_mode_manager:
  ros__parameters:
    control_modes:
      names: [ "idle" ]
      idle:
        type: "empty_control_mode/EmptyControlMode"
```
