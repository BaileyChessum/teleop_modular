# Input source remapping

The logic to do this is currently defined as part of `InputSourceHandle.hpp` and `InputSourceHandle.cpp`.

### Motivation

#### Input renaming

Input sources can export their inputs with whatever name they want.

Why? -- When using joy across different computers, I found controllers would have their inputs be mapped
entirely differently when switching between computers. I needed to rewrite all my input mapping for joy to my control
modes whenever I changed computers.

Instead, I thought it would be handy to be able to define my control mode input names in terms of names of the inputs on
the controller, rather than directly to indices of joy message arrays. Then, as part of the parameters for a
JoyInputSource, I could map the indices to names of the inputs on the controller and load up a different mapping
depending on the computer I was using.

Effectively, I am able to worry about my controller being mapped correctly and making a control scheme for my control
modes separately.

#### Input transformations

This refers to any modifications performed to inputs as part of the `remap:` parameters, such as `invert`, `range`, and
`power`. These allow you to change the behavior of inputs.

This was implemented as part of the InputSourceHandle so that all input source implementations would be able to benefit
from these common utilities.

## Parameters

Parameters for remapping inputs are defined on the node of the input source being remapped.

```yaml
joy_input_source:
  ros__parameters:

    # Defined for all input sources:
    remap:
      axes:
        # Axis remapping definitions go here
        final_axis_name:
          from: "original_axis_name"
          # Transformation params for the input go here
          # ...

      buttons:
        # Button remapping definitions go here
        final_button_name:
          from: "original_button_name"
          # Transformation params for the input go here
          # ...
```

You define the final name of the output under, and specify the original name of the input exported by the input source
to get the value for that input from with the `.from` parameter.

Whenever you define parameters to remap an original input to a different name, the input source will no longer expose
the input under that original name, only the new name.

Note: if you want to apply transformations to an input without renaming it, you still need to define `.from`, just use
the same name.

## Axis Transformations

### Axis `.invert`

When true, the sign of the input value will be flipped.

```yaml
        # Axis remapping definitions go here
        final_axis_name:
          from: "original_axis_name"
          # Transformation params for the input go here
          invert: true
```

### Axis `.range`

This set of parameters allows you to define a linear mapping of input values.

```yaml
        # Axis remapping definitions go here
        final_axis_name:
          from: "original_axis_name"
          # Transformation params for the input go here
          range:
            in: [ -1.0, 1.0 ]
            out: [ 1.0, 0.0 ]
            clamp: true
```

The values in `.in` will become the values in `.out` after mapping.

When `.clamp` is true, values beyond the limits defined in `.in` will be clamped to those extremes (clamping is applied
before mapping to the `.out` range). Clamping can be done without defining `out`.

### Axis `.power`

Sometimes midway values analogue inputs like joysticks just grow too quickly or slowly to be useful. This provides a
means to counteract this by applying an exponent to the input value.

Higher `power` values will bias inputs in the range [0, 1] towards 0,
and lower values of `power` will bias inputs towards 1.

```yaml
        # Axis remapping definitions go here
        final_axis_name:
          from: "original_axis_name"
          # Transformation params for the input go here
          power: 2.0
```

## Button Transformations

### Button `.invert`

When true, true inputs become false, and false inputs become true.

```yaml
        # Button remapping definitions go here
        final_button_name:
          from: "original_button_name"
          # Transformation params for the input go here
          invert: true
```

