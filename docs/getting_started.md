
# Getting started

## Installation

First, you'll need to make sure you have the teleop_modular packages added to your workspace. Currently, you will have 
to build the packages manually. 

> Teleop Modular has been developed using [nix](https://nixos.org/) rather than colcon, so I don't yet serve a prebuilt 
> underlay for you to use. This doesn't mean you need to use nix, or should use nix. 
> For now, I recommend just cloning it into the workspace as a git submodule:
> ```sh
> # Using a colcon workspace (in a git repo!), add teleop_modular as a submodule
> cd workspace/src
> git submodule add https://github.com/BaileyChessum/teleop_modular.git teleop_modular
>
> # Set the revision to avoid unexpected updates to teleop_modular. Set the appropriate version.
> cd teleop_modular
> git checkout tags/v0.1.0
> 
> # Commit your changes to the workspace
> cd ../
> git add ./teleop_modular
> git commit -m "feat: added teleop_modular as a submodule"
> ```
> I will work to make using colcon workspaces easier in the future. If this is your main workflow, I encourage you to 
> contribute improvements to the installation process!

If you're using [nix](https://nixos.org/) with 
[lopsided98/nix-ros-overlay](https://github.com/lopsided98/nix-ros-overlay), add [overlay.nix](../overlay.nix) on top of 
it in your overlays.

---

Once added to your workspace, you should be able to run `teleop_node`:

<pre><span style="background-color:#303030"><font color="#FFFFFF">$ </font></span>./result/bin/ros2 run teleop_node teleop_node

<font color="#C01C28">[ERROR] [teleop_node]: control_modes.names was not set.</font>
[INFO] [teleop_node]: <font color="#2A7BDE"><b>Control Modes:</b></font>

[INFO] [teleop_node]: <font color="#2A7BDE"><b>Input Sources:</b></font>

</pre>

But until we configure `teleop_node`, it won't do anything. 

Follow this guide to start making your own teleop package with the framework: 
- [Writing a teleop package](./writing_a_teleop_package.md)