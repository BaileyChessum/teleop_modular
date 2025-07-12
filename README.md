# teleop_modular

> Note: `teleop_modular` is experimental, and missing build processes and documentation needed for production use. 
>
> As of v0.0.1, you are not yet able to build and use `teleop_modular` without access to a private repository. This will be fixed in the next few days.
> 
> `teleop_modular` will be ready for use shortly. Please star the repository for updates.

`teleop_modular` is a generalized framework for teleoperation input handling in ROS2.

It pairs well with `ros2_control`, supporting switching between multiple different control modes, with associated `ros2_control` controllers. However, it the modular design is generalized to be useful for teleoperation with custom infrastructure for control. 

It aims to replace packages like [teleop-twist-joy](https://github.com/ros-teleop/teleop_twist_joy) and [teleop-twist-keyboard](https://github.com/ros-teleop/teleop_twist_keyboard), with the intent to:
- Prevent control mode logic being tightly coupled to a specific input source.
- Centralize management of multiple control modes.
  - integration with `ros2_control` to dynamically switch active controllers for each control mode.
- Allow new control modes to be easily added to teleop systems using plugins.
  - Promote experimentation with control modes.
  - Allow control mode code to be reused to achieve different functionality 
    (e.g. configuring control modes to use different reference frames).
- Allow novel input sources to be developed and integrated with existing systems
- Improve configuration for individual input sources

More specific WIP documentation and examples can be found under [./teleop_modular/README.md](./teleop_modular/README.md) and [./teleop_modular/docs](./teleop_modular/docs). More information will be provided as designs are finalized after getting feedback from implementing `teleop_modular` into an existing system.


