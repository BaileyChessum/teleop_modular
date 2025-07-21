# Getting started

This set of guides/tutorials will walk through writing complex teleop for a [twist](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)-based robot using Teleop Modular. If you don't have a robot, you should also be able to use [turtlesim](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/basics/ROS2-Turtlesim.html). 

## Prerequisites

This documentation will assume a familiarity with fundamental ROS2 concepts.

If you're new to ROS2, follow these guides from the ROS2 Documentation to get started:

- [ROS2 - Installation](https://docs.ros.org/en/rolling/Installation.html)
- [ROS2 - Configuring environment](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
- [ROS2 - Using colcon to build packages](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [ROS2 - Launching nodes](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html#id5)


## Installation

First, you'll need to make sure you have the teleop_modular packages added to your workspace. Currently, you will have 
to build the packages manually.

> #### Compatibility
> The CI workflow uses Ubuntu (latest) and a colcon workspace to build for:
> - ROS2 Rolling
> - ROS2 Kilted
> - ROS2 Jazzy
> - ROS2 Humble 
> 
> [![CI (Rolling, Jazzy, and Humble)](https://github.com/BaileyChessum/teleop_modular/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/BaileyChessum/teleop_modular/actions/workflows/ci.yml?query=branch%3Amain)
> 
> The package was primarily developed and tested on ROS2 Jazzy with the [nix](https://nixos.org/) package manager using [lopsided98/nix-ros-overlay](https://github.com/lopsided98/nix-ros-overlay) and [hacker1024/nix-ros-workspace](https://github.com/hacker1024/nix-ros-workspace).

#### Building with colcon

> **Note:** I don't yet serve a prebuilt underlay for you to use :(

For now, I recommend just cloning it into the workspace as a git submodule:
```sh
# Using a colcon workspace (in a git repo!), add teleop_modular as a submodule
cd workspace/src
git submodule add https://github.com/BaileyChessum/teleop_modular.git teleop_modular

# Set the revision to avoid unexpected updates to teleop_modular. Set the appropriate version.
cd teleop_modular
git checkout tags/v0.1.0

# Commit your changes to the workspace
cd ../../
git add ./src/teleop_modular
git commit -m "feat: added teleop_modular as a submodule"

# You should now be able to build your workspace
colcon build
. install/setup.bash
```

Please [raise an issue](https://github.com/BaileyChessum/teleop_modular/issues/new) or post in [discussions](https://github.com/BaileyChessum/teleop_modular/discussions/new?category=q-a) if you have issues!

> I will work to make using colcon workspaces easier in the future. If this is your main workflow, I encourage you to 
> contribute improvements to the installation process!

### Building with nix

If you're using [nix](https://nixos.org/) with 
[lopsided98/nix-ros-overlay](https://github.com/lopsided98/nix-ros-overlay), add [overlay.nix](../overlay.nix) on top of 
it in your overlays.

If you don't already use nix, I wouldn't recommend using it to get started with Teleop Modular.

## Running teleop_node

Once added to your workspace, build and source it. You should be able to run `teleop_node`:

```sh
ros2 run teleop_node teleop_node
```

And your output might look like:

<pre>$ ros2 run teleop_node teleop_node

<font color="#C01C28">[ERROR] [teleop_node]: control_modes.names was not set.</font>
[INFO] [teleop_node]: <font color="#2A7BDE"><b>Control Modes:</b></font>

[INFO] [teleop_node]: <font color="#2A7BDE"><b>Input Sources:</b></font>

</pre>

But until we configure `teleop_node`, it won't do anything. 

---

Follow the next guide to start writing your own teleop package with the framework: 
- [Writing a teleop package](./writing_a_teleop_package.md)
