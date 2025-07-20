# Writing a teleop package


## Background

To be able to do anything useful with `teleop_modular`, you'll need to make a package to contain:

- Your launch file to run `teleop_node`
- Your parameter files

# Tasks

## 1. Create a package

Open a terminal and navigate into your workspace src directory. Then, create a package using `ros2 pkg create`. I will 
be creating a C++ package in this tutorial, but a python package should also be fine.

```sh
ros2 pkg create --build-type ament_cmake --license Apache-2.0 teleop_example
```

Replace `teleop_example` with an appropriate name. If I were making a teleop package for a robotic arm, I would name 
this `teleop_arm`.

Now you should have a package to add your launch file to. Your directory structure might look something like this:

```
ws/
└── src/
    └── teleop_example/
        ├── include/
        ├── src/
        ├── CMakeLists.txt
        └── package.xml
```

You can delete `include/` and `src/` if you wish.

### 2. Add directories for launch and parameter files to CMakeLists.txt

By convention, all launch files for a package are stored in the `launch` directory inside of the package. Make sure to 
create a launch directory at the top-level of the package you created above.

```sh
cd teleop_example
mkdir launch
mkdir params
```

To have a package structure that looks like:

```
ws/
└── src/
    └── teleop_example/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        └── params/
```

Then, open `CMakeLists.txt` to add this:

```
# Add all the installation directories
install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)
```

Your final `CMakeLists.txt` might look something like:

```
cmake_minimum_required(VERSION 3.8)
project(teleop_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Add all the installation directories
install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### 3. Create a launch file

Next, we need to add a launch file that

- Runs `teleop_node`
- Passes `teleop_node` the appropriate parameter files

> **Sidenote:** In this tutorial, I will be using python launch files, as you can use python to have the launch file accept some 
interesting arguments that could change what parameter files you want to load. For example, you could load up different configurations for you input sources depending on what controller you wanted to use, such that adding `device:=xbox` or `device:=ps5` would load up the configs for those specific devices. 
> 
> You can also just run `teleop_node` and pass it your parameter files! I do this to quickly test out different 
> parameter file changes without rebuilding the workspace.

Load up your favorite IDE, and create `teleop.launch.py` under `teleop_example/launch/`:

```python
# teleop.launch.py
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution 
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    teleop_example_dir = FindPackageShare('teleop_example')

    teleop_params = LaunchConfiguration('teleop_params')
    log_inputs = LaunchConfiguration('teleop_params')

    return [
        # Runs teleop_node with the given parameter files
        Node(
            package='teleop_node',
            executable='teleop_node',
            output='screen',
            
            # You can add multiple parameter files here:
            parameters=[teleop_params, log_inputs],

            additional_env={
                # Show colors in the terminal output
                'RCUTILS_COLORIZED_OUTPUT': '1', 
                # (Optional!) omit time from the logs
                'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{name}] {message}',
            }
        ),
    ]


def generate_launch_description():
    teleop_example_dir = FindPackageShare('teleop_example')

    declared_arguments = [
        # You can declare arguments to your launch file like this!
        DeclareLaunchArgument(
            name='teleop_params',
            default_value=PathJoinSubstitution([teleop_example_dir, 'params', 'teleop.yaml']),
            description='The main parameter file to use for the teleop_node',
        ),
        DeclareLaunchArgument(
            name='log_inputs',
            default_value='False',
            description='Set this true to display all the inputs. Very useful when trying to configure input sources!',
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
```

Remember to change `teleop_example` to the name of your package.

We will expand on this launch file later as we add input sources and control modes.

## 4. Create the teleop.yaml parameter file

Create a teleop.yaml file to configure the behavior of the core teleop_node.

```yaml
# teleop.yaml
teleop_node:
  ros__parameters:
    # The maximum rate at which updates should occur, and hence the max rate at which commands are sent. 
    # Leaving this unset makes the max update rate unlimited. 
    update_rate: 50.0
    
    # The minimum rate at which updates occur. 
    # Leaving this unset will allow for indefinite lapses between updates.
    min_update_rate: 1.0
```

We'll add control modes and input sources later.

For now, you should have everything you need to run your teleop package. Try to build and source your workspace, then run 
the launch file:

```sh
ros2 launch teleop_example teleop.launch.py
```

Your output might look like:

<pre>$ ros2 launch teleop_example teleop.launch.py log_inputs:=true
[INFO] [launch]: All log files can be found below /home/nova/.ros/log/2025-07-20-23-34-41-464329-nixos-2142296
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [teleop_node-1]: process started with pid [2142306]
[teleop_node-1] <font color="#C01C28">[ERROR] [teleop_node] control_modes.names was not set.</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#2A7BDE"><b>Control Modes:</b></font>
[teleop_node-1] 
[teleop_node-1] [INFO] [teleop_node] <font color="#2A7BDE"><b>Input Sources:</b></font>
[teleop_node-1] 
</pre>

### 5. Add a control mode

In `teleop.yaml` define a control mode. I'll be using [teleop_modular_twist/TwistControlMode](../teleop_modular_twist).

```yaml
# teleop.yaml
teleop_node:
  ros__parameters:
    # ... 

    # Add this:
    control_modes:
      names: [ 
        # Give your control mode a name!
        "twist_control_mode"
      ]
      
      # Then declare its type!
      twist_control_mode:
        type: "teleop_modular_twist/TwistControlMode"
```

> **Note**: For `ros2_control` users: if you also want Teleop Modular to activate and deactivate controllers in `ros2_control` alongside your control modes,
> you can add the names of the controllers you want to be activated with the control mode under the `controllers` 
> parameter:
> ```yaml
>      # ...
>      twist_control_mode:
>        type: "teleop_modular_twist/TwistControlMode"
>        controllers: [
>          "some_ros2_control_controller_name"
>        ]
> ```
> 

Then, we need to define parameters for the node created for `twist_control_mode`. You can either add this at the bottom 
of `teleop.yaml`, or make a new parameter file. I will just be adding them to the end of `teleop.yaml` in this tutorial.

```yaml
# teleop.yaml
teleop_node:
  ros__parameters:
    # ...

twist_control_mode:
  ros__parameters:
    # TwistStamped messages will be published on this
    topic: "/twist"
    max_speed:
      linear: 0.25
      angular: 0.25
```

Check the docs for the control mode you use to find out what parameters exist for it.

Then, try running it!

```sh
ros2 launch teleop_example teleop.launch.py teleop_params:=/path/to/ws/src/teleop_example/params/teleop.yaml
```

> **Note:** we don't need to rebuild the workspace to try out changes to the config, as long as we specify the absolute 
> path to the config file. Specify the absolute path to the config file whenever you want to actively edit your parameter 
> files.

Your output should no longer have the error for missing the `control_modes.names` parameter, and your control mode 
should be listed.

<pre>$ ros2 launch teleop_arm teleop.launch.py teleop_params:=/home/.../teleop_example/params/teleop.yaml
[INFO] [launch]: All log files can be found below /home/nova/.ros/log/2025-07-21-01-35-16-545322-nixos-2213788
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [teleop_node-1]: process started with pid [2213791]
[teleop_node-1] [INFO] [teleop_node] <font color="#2A7BDE"><b>Control Modes:</b></font>
[teleop_node-1] 	- Twist Control Mode	<font color="#5D5D5D">: teleop_modular_twist/TwistControlMode</font>
[teleop_node-1] 
[teleop_node-1] [INFO] [teleop_node] <font color="#2A7BDE"><b>Input Sources:</b></font>
[teleop_node-1] 
[teleop_node-1] [INFO] [teleop_node] <font color="#A347BA">Twist Control Mode activated</font>
</pre>

If you have any issues, please post in 
[Discussions](https://github.com/BaileyChessum/teleop_modular/discussions/new?category=q-a), and I will try to help!

## 6. Add an input source

Adding an input source is a very similar process to the previous step.

In `teleop.yaml` define an input source. I'll be using [teleop_modular_joy/JoyInputSource](../teleop_modular_joy).

```yaml
# teleop.yaml
teleop_node:
  ros__parameters:
    # ... 

    # Add this:
    input_sources:
      names: [
        # Give your input source a name!
        "joy_input_source"
      ]

      # Then declare its type!
      joy_input_source:
        type: "teleop_modular_joy/JoyInputSource"
```

Then, we need to define parameters for the node created for `joy_input_source`. You can either add this at the bottom
of `teleop.yaml`, or make a new parameter file. I will just be adding them to the end of `teleop.yaml` in this tutorial.

```yaml
# teleop.yaml
teleop_node:
  ros__parameters:
    # ...

joy_input_source:
  ros__parameters:
    topic: "/joy"
    # These are the default mappings for a game_controller_node.
    # https://docs.ros.org/en/ros2_packages/rolling/api/joy/index.html
    # If you use joy_node, you'll need to figure out what the mapping is yourself. 
    # So, run joy, `ros2 topic echo /joy`, and mess around with your controller to figure out what it is for you.
    button_definitions: [
      "A",
      "B",
      "X",
      "Y",
      "BACK",
      "GUIDE",
      "START",
      "LEFTSTICK",
      "RIGHTSTICK",
      "LEFTSHOULDER",
      "RIGHTSHOULDER",
      "DPAD_UP",
      "DPAD_DOWN",
      "DPAD_LEFT",
      "DPAD_RIGHT",
      "MISC1",
      "PADDLE1",
      "PADDLE2",
      "PADDLE3",
      "PADDLE4",
      "TOUCHPAD"
    ]
    axis_definitions: [
      "LEFTX",
      "LEFTY",
      "RIGHTX",
      "RIGHTY",
      "TRIGGERLEFT",
      "TRIGGERRIGHT"
    ]
```

Check the docs for the input source you use to find out what parameters exist for it.

In the example above, the joy_input_source will export all the axis and button names listed, associated with the 
values from Joy messages, maintaining the same order as the list of names.

Now, try running it!

```sh
ros2 launch teleop_example teleop.launch.py teleop_params:=/path/to/ws/src/teleop_example/params/teleop.yaml
```

<pre>$ ros2 launch teleop_example teleop.launch.py teleop_params:=/home/.../teleop_example/params/teleop.yaml
[INFO] [launch]: All log files can be found below /home/nova/.ros/log/2025-07-21-02-08-53-614455-nixos-2229263
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [teleop_node-1]: process started with pid [2229273]
[teleop_node-1] [INFO] [teleop_node] <font color="#2A7BDE"><b>Control Modes:</b></font>
[teleop_node-1] 	- Twist Control Mode	<font color="#5D5D5D">: teleop_modular_twist/TwistControlMode</font>
[teleop_node-1] 
[teleop_node-1] [INFO] [teleop_node] <font color="#2A7BDE"><b>Input Sources:</b></font>
[teleop_node-1] 	- Joy Input Source	<font color="#5D5D5D">: teleop_modular_joy/JoyInputSource</font>
[teleop_node-1] 
[teleop_node-1] [INFO] [teleop_node] <font color="#A347BA">Twist Control Mode activated</font>
</pre>

You should now see your input source listed.

If you want to see your inputs, you can set the parameter `log_inputs:=True`:

```sh
ros2 launch teleop_example teleop.launch.py ... log_inputs:=true
```

Plug in a controller, and run in another terminal:

```sh 
ros2 run joy game_controller_node
```

Mess around with controller inputs, and you should see them appear in your original terminal:

<pre>$ ros2 launch teleop_example teleop.launch.py teleop_params:=/home/.../teleop_example/params/teleop.yaml log_inputs:=true
[INFO] [launch]: All log files can be found below /home/nova/.ros/log/2025-07-21-02-16-07-117871-nixos-2232880
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [teleop_node-1]: process started with pid [2232890]
[teleop_node-1] [INFO] [teleop_node] <font color="#2A7BDE"><b>Control Modes:</b></font>
[teleop_node-1] 	- Twist Control Mode	<font color="#5D5D5D">: teleop_modular_twist/TwistControlMode</font>
[teleop_node-1] 
[teleop_node-1] [INFO] [teleop_node] <font color="#2A7BDE"><b>Input Sources:</b></font>
[teleop_node-1] 	- Joy Input Source	<font color="#5D5D5D">teleop_modular_joy/JoyInputSource</font>
[teleop_node-1] 
[teleop_node-1] [INFO] [teleop_node] <font color="#A347BA">Twist Control Mode activated</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  RIGHTY</font>	<font color="#A2734C">0.116634</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  RIGHTX</font>	<font color="#A2734C">-0.306491</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  RIGHTY</font>	<font color="#A2734C">0.744094</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  RIGHTX</font>	<font color="#A2734C">-0.504636</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  RIGHTY</font>	<font color="#A2734C">0.983520</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  RIGHTX</font>	<font color="#A2734C">-0.364283</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  RIGHTX</font>	<font color="#A2734C">-0.124858</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  RIGHTX</font>	<font color="#A2734C">-0.017529</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  RIGHTY</font>	<font color="#A2734C">0.801887</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  RIGHTY</font>	<font color="#A2734C">0.636765</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  RIGHTY</font>	<font color="#A2734C">0.455132</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  RIGHTY</font>	<font color="#A2734C">0.034073</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  DPAD_RIGHT</font>	<font color="#A2734C">1</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  DPAD_RIGHT</font>	<font color="#A2734C">0</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  DPAD_LEFT</font>	<font color="#A2734C">1</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  DPAD_LEFT</font>	<font color="#A2734C">0</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  DPAD_DOWN</font>	<font color="#A2734C">1</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  DPAD_DOWN</font>	<font color="#A2734C">0</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  DPAD_UP</font>	<font color="#A2734C">1</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  DPAD_UP</font>	<font color="#A2734C">0</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  DPAD_RIGHT</font>	<font color="#A2734C">1</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  DPAD_RIGHT</font>	<font color="#A2734C">0</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  DPAD_LEFT</font>	<font color="#A2734C">1</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  DPAD_LEFT</font>	<font color="#A2734C">0</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  A</font>	<font color="#A2734C">1</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  A</font>	<font color="#A2734C">0</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  B</font>	<font color="#A2734C">1</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  B</font>	<font color="#A2734C">0</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  B</font>	<font color="#A2734C">1</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  B</font>	<font color="#A2734C">0</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  Y</font>	<font color="#A2734C">1</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  Y</font>	<font color="#A2734C">0</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  X</font>	<font color="#A2734C">1</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  X</font>	<font color="#A2734C">0</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  TRIGGERRIGHT</font>	<font color="#A2734C">-0.252811</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  TRIGGERRIGHT</font>	<font color="#A2734C">-0.529405</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  TRIGGERRIGHT</font>	<font color="#A2734C">-0.706894</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  TRIGGERRIGHT</font>	<font color="#A2734C">-0.872015</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  TRIGGERRIGHT</font>	<font color="#A2734C">-1.000000</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  TRIGGERRIGHT</font>	<font color="#A2734C">-0.471612</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  TRIGGERRIGHT</font>	<font color="#A2734C">-0.050553</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  TRIGGERLEFT</font>	<font color="#A2734C">-0.203274</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  TRIGGERLEFT</font>	<font color="#A2734C">-0.686270</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  TRIGGERLEFT</font>	<font color="#A2734C">-1.000000</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  TRIGGERLEFT</font>	<font color="#A2734C">-0.884415</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  TRIGGERLEFT</font>	<font color="#A2734C">-0.323003</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  TRIGGERLEFT</font>	<font color="#A2734C">-0.000000</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  LEFTSHOULDER</font>	<font color="#A2734C">1</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  LEFTSHOULDER</font>	<font color="#A2734C">0</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  RIGHTSHOULDER</font>	<font color="#A2734C">1</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  RIGHTSHOULDER</font>	<font color="#A2734C">0</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  GUIDE</font>	<font color="#A2734C">1</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  GUIDE</font>	<font color="#A2734C">0</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  START</font>	<font color="#A2734C">1</font>
[teleop_node-1] [INFO] [teleop_node] <font color="#A2734C">  START</font>	<font color="#A2734C">0</font>
</pre>

You could also change your launch file to run `game_controller_node` or `joy_node` alongside `teleop_node`:

```python
# teleop.launch.py
# ...
def launch_setup(context, *args, **kwargs):
    # ...
    return [
        # Automatically run joy alongside teleop
        Node(
            package='joy',
            executable='game_controller_node',  # or joy_node
            output="screen"
        ),
        # Runs teleop_node with the given parameter files
        Node(
            package='teleop_node',
            executable='teleop_node',
            # ...
        ),
    ]

# ...
```

If you have any issues, please post in
[Discussions](https://github.com/BaileyChessum/teleop_modular/discussions/new?category=q-a), and I will try to help!

## 7. Mapping inputs

Congrats! You've reached the fun part.

Currently, your control mode isn't getting any inputs from the input source. If we check the documentation for [teleop_modular_twist/TwistControlMode](../teleop_modular_twist), we'll find that it 
expects to get these axis inputs:

- `speed`: The input axis that scales the output speed from 0 to 1.
- `twist_x`: The input axis providing the x component of the twist linear velocity from -1 to 1.
- `twist_y`: The input axis providing the y component of the twist linear velocity from -1 to 1.
- `twist_z`: The input axis providing the z component of the twist linear velocity from -1 to 1.
- `twist_roll`: The input axis providing the x component of the twist angular velocity from -1 to 1.
- `twist_pitch`: The input axis providing the y component of the twist angular velocity from -1 to 1.
- `twist_yaw`: The input axis providing the z component of the twist angular velocity from -1 to 1.

And note, if you don't set `speed`, it will default to 0, so nothing will happen.

You could change the names of the inputs in `joy_input_source`'s `axis_definitions` parameter to match those needed by 
the control mode. But, chances are, the inputs aren't behaving exactly as you'd like. Some input axes might be inverted, 
for example. 

Teleop Modular solves this problem with the set of [remap parameters](./input_source_remapping.md) automatically added 
to every input source implementation. We can use them to:
- Rename inputs
- Create axes from buttons
- Create buttons from axes
- Transform input values in various ways, such as:
  - Inverting axes and buttons
  - Linearly mapping an input range of axis values to an output range
  - Clamping axes

We will apply these parameters to solve our problem in the following guide:
- [Remapping and transforming inputs](./remapping_and_transforming_inputs.md)
