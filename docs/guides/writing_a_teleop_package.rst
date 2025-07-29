.. _writing_a_teleop_package:

Writing a teleop package
========================

**Goal**: Create a package to run a custom configuration of teleop_modular.

**Tutorial level**: Intermediate

**Time**: 40 minutes

Background
----------

To be able to do anything useful with ``teleop_modular``, you'll need to make a package to contain:

- Your launch file to run ``teleop_node``
- Your parameter files

Prerequisites
-------------

You should have ``teleop_modular`` installed in your workspace.

- :ref:`getting_started`

Tasks
-----

1. Create a package
^^^^^^^^^^^^^^^^^^^

Open a terminal and navigate into your workspace src directory. Then, create a package using ``ros2 pkg create``. I will
be creating a C++ package in this tutorial, but a python package should also be fine.

.. code-block:: sh

   ros2 pkg create --build-type ament_cmake --license Apache-2.0 teleop_example

Replace ``teleop_example`` with an appropriate name. If I were making a teleop package for a robotic arm, I would name
this ``teleop_arm``.

Now you should have a package to add your launch file to. Your directory structure might look something like this:

.. code-block:: none

   ws/
   └── src/
       └── teleop_example/
           ├── include/
           ├── src/
           ├── CMakeLists.txt
           └── package.xml

You can delete ``include/`` and ``src/`` if you wish.

2. Add directories for launch and parameter files to CMakeLists.txt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By convention, all launch files for a package are stored in the ``launch`` directory inside of the package. Make sure to
create a launch directory at the top-level of the package you created above.

.. code-block:: sh

   cd teleop_example
   mkdir launch
   mkdir params

To have a package structure that looks like:

.. code-block:: none

   ws/
   └── src/
       └── teleop_example/
           ├── CMakeLists.txt
           ├── package.xml
           ├── launch/
           └── params/

Then, open ``CMakeLists.txt`` to add this:

.. code-block:: cmake

   # Add all the installation directories
   install(DIRECTORY
     launch
     params
     DESTINATION share/${PROJECT_NAME}
   )

Your final ``CMakeLists.txt`` might look something like:

.. code-block:: cmake

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

3. Create a launch file
^^^^^^^^^^^^^^^^^^^^^^^

Next, we need to add a launch file that

- Runs ``teleop_node`` with a unique name
- Passes ``teleop_node`` the appropriate parameter files

.. admonition:: Sidenote

   In this tutorial, I will be using python launch files, as you can use python to have the launch file
   accept some interesting arguments that could change what parameter files you want to load. For example, you could load
   up different configurations for your input sources depending on what controller you wanted to use, such that adding
   ``device:=xbox`` or ``device:=ps5`` would load up the configs for those specific devices.

   You can also just run ``teleop_node`` and pass it your parameter files! I do this to quickly test out different
   parameter file changes without rebuilding the workspace.

Load up your favorite IDE, and create ``teleop.launch.py`` under ``teleop_example/launch/``:

.. code-block:: python

   # teleop.launch.py
   from launch import LaunchDescription
   from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
   from launch.actions import DeclareLaunchArgument, OpaqueFunction
   from launch_ros.actions import Node
   from launch_ros.substitutions import FindPackageShare
   from launch_ros.parameter_descriptions import ParameterValue

   def launch_setup(context, *args, **kwargs):
       teleop_example_dir = FindPackageShare('teleop_example')

       teleop_params = LaunchConfiguration('teleop_params')
       log_inputs = LaunchConfiguration('log_inputs')

       return [
           Node(
               package='teleop_node',
               executable='teleop_node',
               output='screen',
               arguments=['--node-name', 'teleop_example'],
               parameters=[
                   teleop_params,
                   {'log_inputs': ParameterValue(log_inputs, value_type=bool)}
               ],
               additional_env={
                   'RCUTILS_COLORIZED_OUTPUT': '1',
                   'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{name}] {message}',
               }
           ),
       ]

   def generate_launch_description():
       teleop_example_dir = FindPackageShare('teleop_example')

       declared_arguments = [
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

Remember to change ``teleop_example`` to the name of your package.

We will expand on this launch file later as we add input sources and control modes.

4. Create the teleop.yaml parameter file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a teleop.yaml file to configure the behavior of the core teleop_node.

.. code-block:: yaml

   # teleop.yaml
   teleop_example:
     ros__parameters:
       # The maximum rate at which updates should occur, and hence the max rate at which commands are sent.
       # Leaving this unset makes the max update rate unlimited.
       # update_rate: 50.0

       # The minimum rate at which updates occur.
       # Leaving this unset will allow for indefinite lapses between updates.
       min_update_rate: 2.0

       # Note: all update rates are in hz

.. note::

   Make sure you use the same name you gave the node in the launch file at the root of the yaml.

We'll add control modes and input sources later.

For now, you should have everything you need to run your teleop package. Try to build and source your workspace, then run the launch file:

.. code-block:: sh

   ros2 launch teleop_example teleop.launch.py

Your output might look like:

.. code-block:: console

   $ ros2 launch teleop_example teleop.launch.py
   [INFO] [launch]: All log files can be found below /home/nova/.ros/log/2025-07-20-23-34-41-464329-nixos-2142296
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [teleop_node-1]: process started with pid [2142306]
   [teleop_node-1] [ERROR] [teleop_example] control_modes.names was not set.
   [teleop_node-1] [INFO] [teleop_example] Control Modes:
   [teleop_node-1]
   [teleop_node-1] [INFO] [teleop_example] Input Sources:
   [teleop_node-1]

5. Add a control mode
^^^^^^^^^^^^^^^^^^^^^

In ``teleop.yaml`` define a control mode. I'll be using `teleop_modular_twist/TwistControlMode <../teleop_modular_twist>`_.

.. code-block:: yaml

   # teleop.yaml
   teleop_example:
     ros__parameters:
       # ...

       # Add this:
       control_modes:
         names: [
           # Give your control mode a name! Use snake_case
           "twist_control_mode"
         ]

         # Then declare its type!
         twist_control_mode:
           type: "teleop_modular_twist/TwistControlMode"

.. note::

   For ``ros2_control`` users: if you also want Teleop Modular to activate and deactivate controllers in ``ros2_control`` alongside your control modes,
   you can add the names of the controllers you want to be activated with the control mode under the ``controllers`` parameter:

   .. code-block:: yaml

      # ...
      twist_control_mode:
        type: "teleop_modular_twist/TwistControlMode"
        controllers: [
          "some_ros2_control_controller_name"
        ]

Then, we need to define parameters for the node created for ``twist_control_mode``. You can either add this at the bottom of ``teleop.yaml``, or make a new parameter file. I will just be adding them to the end of ``teleop.yaml`` in this tutorial.

.. code-block:: yaml

   # teleop.yaml
   teleop_example:
     ros__parameters:
       # ...

   # Add this:
   twist_control_mode:
     ros__parameters:
       # Twist messages will be published on this
       topic: "/twist"
       # This will disable the input called 'speed', which multiplies every other input when enabled
       use_speed_input: false

       scale:
         linear:
           all: 2.0  # max speed of 2 meters per second
         angular:
           all: 2.0  # max angular speed of 2 radians per second

       # Check the README.md for teleop_modular_twist/TwistControlMode for more parameters!
       # It is very extensive.
       # https://github.com/BaileyChessum/teleop_modular/tree/main/teleop_modular_twist

Check the docs for the control mode you use to find out what parameters exist for it.

Then, try running it!

.. code-block:: sh

   ros2 launch teleop_example teleop.launch.py teleop_params:=/path/to/ws/src/teleop_example/params/teleop.yaml

.. note::

   We don't need to rebuild the workspace to try out changes to the config, as long as we specify the absolute
   path to the config file. Specify the absolute path to the config file whenever you want to actively edit your
   parameter files.

Your output should no longer have the error for missing the ``control_modes.names`` parameter, and your control mode should be listed.

.. code-block:: console

   $ ros2 launch teleop_arm teleop.launch.py teleop_params:=/home/.../teleop_example/params/teleop.yaml
   [INFO] [launch]: All log files can be found below /home/nova/.ros/log/2025-07-21-01-35-16-545322-nixos-2213788
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [teleop_node-1]: process started with pid [2213791]
   [teleop_node-1] [INFO] [teleop_example] Control Modes:
   [teleop_node-1] 	- Twist Control Mode	: teleop_modular_twist/TwistControlMode
   [teleop_node-1]
   [teleop_node-1] [INFO] [teleop_example] Input Sources:
   [teleop_node-1]
   [teleop_node-1] [INFO] [teleop_example] Twist Control Mode activated

If you have any issues, please post in
`Discussions <https://github.com/BaileyChessum/teleop_modular/discussions/new?category=q-a>`_, and I will try to help!

6. Add an input source
^^^^^^^^^^^^^^^^^^^^^^

Adding an input source is a very similar process to the previous step.

In ``teleop.yaml`` define an input source. I'll be using :class:`teleop_modular_joy/JoyInputSource <../teleop_modular_joy>`.

.. code-block:: yaml

   # teleop.yaml
   teleop_example:
     ros__parameters:
       # ...

       # Add this:
       input_sources:
         names: [
           # Give your input source a name! Use snake_case
           "joy_input_source"
         ]

         # Then declare its type!
         joy_input_source:
           type: "teleop_modular_joy/JoyInputSource"

Then, we need to define parameters for the node created for ``joy_input_source``. You can either add this at the bottom
of ``teleop.yaml``, or make a new parameter file. I will just be adding them to the end of ``teleop.yaml`` in this tutorial.

.. code-block:: yaml

   # teleop.yaml
   teleop_example:
     ros__parameters:
     # ...

   # Add this:
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

Check the docs for the input source you use to find out what parameters exist for it.

In the example above, the ``joy_input_source`` will export all the axis and button names listed, associated with the
values from Joy messages, maintaining the same order as the list of names.

Now, try running it!

.. code-block:: console

   $ ros2 launch teleop_example teleop.launch.py teleop_params:=/path/to/ws/src/teleop_example/params/teleop.yaml

.. parsed-literal::

   $ ros2 launch teleop_example teleop.launch.py teleop_params:=/home/.../teleop_example/params/teleop.yaml
   [INFO] [launch]: All log files can be found below /home/nova/.ros/log/2025-07-21-02-08-53-614455-nixos-2229263
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [teleop_node-1]: process started with pid [2229273]
   [teleop_node-1] [INFO] [teleop_node] Control Modes:
   [teleop_node-1] 	- Twist Control Mode	: teleop_modular_twist/TwistControlMode
   [teleop_node-1]
   [teleop_node-1] [INFO] [teleop_node] Input Sources:
   [teleop_node-1] 	- Joy Input Source	: teleop_modular_joy/JoyInputSource
   [teleop_node-1]
   [teleop_node-1] [INFO] [teleop_node] Twist Control Mode activated

You should now see your input source listed.

If you want to see your inputs, you can set the parameter ``log_inputs:=True``:

.. code-block:: console

   $ ros2 launch teleop_example teleop.launch.py ... log_inputs:=true

Plug in a controller, and run in another terminal:

.. code-block:: console

   $ ros2 run joy game_controller_node

Mess around with controller inputs, and you should see them appear in your original terminal:

.. code-block:: console

   $ ros2 launch teleop_example teleop.launch.py teleop_params:=/home/.../teleop_example/params/teleop.yaml log_inputs:=True
   [INFO] [launch]: All log files can be found below /home/nova/.ros/log/2025-07-21-02-16-07-117871-nixos-2232880
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [teleop_node-1]: process started with pid [2232890]
   [teleop_node-1] [INFO] [teleop_example] Control Modes:
   [teleop_node-1] 	- Twist Control Mode	: teleop_modular_twist/TwistControlMode
   [teleop_node-1]
   [teleop_node-1] [INFO] [teleop_example] Input Sources:
   [teleop_node-1] 	- Joy Input Source	: teleop_modular_joy/JoyInputSource
   [teleop_node-1]
   [teleop_node-1] [INFO] [teleop_example] Twist Control Mode activated
   [teleop_node-1] [INFO] [teleop_example]   RIGHTY	0.116634
   [teleop_node-1] [INFO] [teleop_example]   RIGHTX	-0.306491
   [teleop_node-1] [INFO] [teleop_example]   RIGHTY	0.744094
   [teleop_node-1] [INFO] [teleop_example]   RIGHTX	-0.504636
   [teleop_node-1] [INFO] [teleop_example]   RIGHTY	0.983520
   [teleop_node-1] [INFO] [teleop_example]   RIGHTX	-0.364283
   [teleop_node-1] [INFO] [teleop_example]   RIGHTX	-0.124858
   [teleop_node-1] [INFO] [teleop_example]   RIGHTX	-0.017529
   [teleop_node-1] [INFO] [teleop_example]   RIGHTY	0.801887
   [teleop_node-1] [INFO] [teleop_example]   RIGHTY	0.636765
   [teleop_node-1] [INFO] [teleop_example]   RIGHTY	0.455132
   [teleop_node-1] [INFO] [teleop_example]   RIGHTY	0.034073
   [teleop_node-1] [INFO] [teleop_example]   DPAD_RIGHT	1
   [teleop_node-1] [INFO] [teleop_example]   DPAD_RIGHT	0
   [teleop_node-1] [INFO] [teleop_example]   DPAD_LEFT	1
   [teleop_node-1] [INFO] [teleop_example]   DPAD_LEFT	0
   [teleop_node-1] [INFO] [teleop_example]   DPAD_DOWN	1
   [teleop_node-1] [INFO] [teleop_example]   DPAD_DOWN	0
   [teleop_node-1] [INFO] [teleop_example]   DPAD_UP	1
   [teleop_node-1] [INFO] [teleop_example]   DPAD_UP	0
   [teleop_node-1] [INFO] [teleop_example]   DPAD_RIGHT	1
   [teleop_node-1] [INFO] [teleop_example]   DPAD_RIGHT	0
   [teleop_node-1] [INFO] [teleop_example]   DPAD_LEFT	1
   [teleop_node-1] [INFO] [teleop_example]   DPAD_LEFT	0
   [teleop_node-1] [INFO] [teleop_example]   A	1
   [teleop_node-1] [INFO] [teleop_example]   A	0
   [teleop_node-1] [INFO] [teleop_example]   B	1
   [teleop_node-1] [INFO] [teleop_example]   B	0
   [teleop_node-1] [INFO] [teleop_example]   B	1
   [teleop_node-1] [INFO] [teleop_example]   B	0
   [teleop_node-1] [INFO] [teleop_example]   Y	1
   [teleop_node-1] [INFO] [teleop_example]   Y	0
   [teleop_node-1] [INFO] [teleop_example]   X	1
   [teleop_node-1] [INFO] [teleop_example]   X	0
   [teleop_node-1] [INFO] [teleop_example]   TRIGGERRIGHT	-0.252811
   [teleop_node-1] [INFO] [teleop_example]   TRIGGERRIGHT	-0.529405
   [teleop_node-1] [INFO] [teleop_example]   TRIGGERRIGHT	-0.706894
   [teleop_node-1] [INFO] [teleop_example]   TRIGGERRIGHT	-0.872015
   [teleop_node-1] [INFO] [teleop_example]   TRIGGERRIGHT	-1.000000
   [teleop_node-1] [INFO] [teleop_example]   TRIGGERRIGHT	-0.471612
   [teleop_node-1] [INFO] [teleop_example]   TRIGGERRIGHT	-0.050553
   [teleop_node-1] [INFO] [teleop_example]   TRIGGERLEFT	-0.203274
   [teleop_node-1] [INFO] [teleop_example]   TRIGGERLEFT	-0.686270
   [teleop_node-1] [INFO] [teleop_example]   TRIGGERLEFT	-1.000000
   [teleop_node-1] [INFO] [teleop_example]   TRIGGERLEFT	-0.884415
   [teleop_node-1] [INFO] [teleop_example]   TRIGGERLEFT	-0.323003
   [teleop_node-1] [INFO] [teleop_example]   TRIGGERLEFT	-0.000000
   [teleop_node-1] [INFO] [teleop_example]   LEFTSHOULDER	1
   [teleop_node-1] [INFO] [teleop_example]   LEFTSHOULDER	0
   [teleop_node-1] [INFO] [teleop_example]   RIGHTSHOULDER	1
   [teleop_node-1] [INFO] [teleop_example]   RIGHTSHOULDER	0
   [teleop_node-1] [INFO] [teleop_example]   GUIDE	1
   [teleop_node-1] [INFO] [teleop_example]   GUIDE	0
   [teleop_node-1] [INFO] [teleop_example]   START	1
   [teleop_node-1] [INFO] [teleop_example]   START	0

You could also change your launch file to run ``game_controller_node`` or ``joy_node`` alongside ``teleop_node``:

.. code-block:: python

   # teleop.launch.py
   # ...
   def launch_setup(context, *args, **kwargs):
       # ...
       return [
           # Add this!
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

If you have any issues, please post in
`Discussions <https://github.com/BaileyChessum/teleop_modular/discussions/new?category=q-a>`_, and I will try to help!

7. Mapping inputs
^^^^^^^^^^^^^^^^^

Congrats! You've reached the fun part.

Currently, your control mode isn't getting any inputs from the input source. If we check the documentation
for :class:`teleop_modular_twist/TwistControlMode <../teleop_modular_twist>`, we'll find that it
expects to get these axis inputs:

- ``linear.x``: The input axis providing the x component of the twist linear velocity from -1 to 1.
- ``linear.y``: The input axis providing the y component of the twist linear velocity from -1 to 1.
- ``linear.z``: The input axis providing the z component of the twist linear velocity from -1 to 1.
- ``angular.x``: The input axis providing the x component of the twist angular velocity from -1 to 1.
- ``angular.y``: The input axis providing the y component of the twist angular velocity from -1 to 1.
- ``angular.z``: The input axis providing the z component of the twist angular velocity from -1 to 1.

- ``speed``: The input axis that scales the output speed from 0 to 1. But, since we set ``use_speed_input: false`` in the parameter file, we shouldn't need to set this.

You could change the names of the inputs in ``joy_input_source``'s ``axis_definitions`` parameter to match those needed by
the control mode. But, chances are, the inputs aren't behaving exactly as you'd like. Some input axes might be inverted,
for example.

Teleop Modular solves this problem with the set of :ref:`remap parameters <input_source_remapping>` automatically added
to every input source implementation. We can use them to:

* Rename inputs
* Create axes from buttons
* Create buttons from axes
* Transform input values in various ways, such as:

   * Inverting axes and buttons
   * Linearly mapping an input range of axis values to an output range
   * Clamping axes

We will apply these parameters to solve our problem in the following guide:

- :ref:`remapping_and_transforming_inputs`