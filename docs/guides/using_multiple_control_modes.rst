.. _using_multiple_control_modes:

Using multiple control modes
============================

**Goal**: Run multiple control modes and switch between them at runtime.

**Tutorial level**: Intermediate

**Time**: 20 minutes

Background
----------

A single ``teleop_node`` can manage several control modes simultaneously. Each control
mode has its own node, parameters, and set of ros2_control controllers it manages.
At runtime, each mode can be independently activated or deactivated -- you choose which
modes are active and configure commands to switch between them on button events.

A typical use case is a robotic arm with two operating modes: a *joint space* mode
that commands individual joint velocities, and a *task space* mode that commands
end-effector motion via inverse kinematics. The operator holds a button to enter task
space mode and releases it to return to joint space mode.

Prerequisites
-------------

- :ref:`writing_a_teleop_package`
- :ref:`writing_a_control_mode`

Tasks
-----

1. Declare your control modes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

List all control modes under the ``control_modes.names`` parameter. Each mode needs
a ``type`` (the pluginlib class name) and optionally a ``display_name``.

.. code-block:: yaml

   teleop_arm:
     ros__parameters:
       # ...
       control_modes:
         names:
           - joint_space_mode
           - task_space_mode
           - task_space_velocity_mode
           - end_effector_mode

         joint_space_mode:
           display_name: "Joint Space"
           type: "joint_space_control_mode/JointSpaceControlMode"
           active: true
           controllers:
             - nova_arm_velocity_controller

         task_space_mode:
           display_name: "Task Space (IK)"
           type: "teleop_modular_twist/TwistControlMode"
           controllers:
             - "nova_twistmapper"
             - "nova_arm_position_controller"

         task_space_velocity_mode:
           display_name: "Task Space (Velocity IK)"
           type: "teleop_modular_twist/TwistControlMode"
           controllers:
             - "nova_twistmapper_velocity"
             - "nova_arm_velocity_controller"

         end_effector_mode:
           display_name: "End Effector"
           type: "joint_space_control_mode/JointSpaceControlMode"
           active: true
           controllers:
             - nova_end_effector_velocity_controller

- ``active: true`` -- the mode starts active when the node launches. Modes without
  this parameter start inactive.
- ``controllers`` -- the ros2_control controller names to activate/deactivate along with
  this mode. When the mode activates, these controllers are started; when it deactivates
  they are stopped.

2. Configure each control mode
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Each control mode reads its parameters from a top-level node named after the mode.
Add parameter blocks for each mode in your YAML:

.. code-block:: yaml

   joint_space_mode:
     ros__parameters:
       topic: "/arm/joint_velocity_command"
       use_speed_input: true
       joint_definitions:
         - "j1"
         - "j2"
         - "j3"
         - "j4"
         - "j5"
         - "j6"
       joints:
         j1:
           input_name: "j1"
           scale: 0.188
         # ...

   task_space_mode:
     ros__parameters:
       stamped_topic: "/arm/task_space_twist_command"
       use_speed_input: true
       scale:
         linear:
           all: 0.05
         angular:
           all: 0.3
       limits:
         linear:
           all: 0.05
           normalized: true
           scale_with_speed: true
         angular:
           all: 0.1
           normalized: true
           scale_with_speed: true

3. Add mode-switching commands
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use ``switch_control_mode`` commands to activate and deactivate modes in response to
button events. Each command specifies which modes to ``activate`` and which to
``deactivate``:

.. code-block:: yaml

   teleop_arm:
     ros__parameters:
       # ...
       commands:
         names:
           - enter_task_space
           - enter_task_space_velocity
           - exit_task_space_velocity
           - enter_joint_space

         enter_task_space:
           on: "task_space/down"
           type: switch_control_mode
           activate:
             - "task_space_mode"
           deactivate:
             - "joint_space_mode"

         enter_task_space_velocity:
           on: "task_space_velocity/down"
           type: switch_control_mode
           activate:
             - "task_space_velocity_mode"
           deactivate:
             - "joint_space_mode"

         exit_task_space_velocity:
           on_any:
             - "task_space_velocity/up"
           type: switch_control_mode
           activate:
             - "joint_space_mode"
           deactivate:
             - "task_space_velocity_mode"

         enter_joint_space:
           on_any:
             - "task_space/up"
           type: switch_control_mode
           activate:
             - "joint_space_mode"
           deactivate:
             - "task_space_mode"

This creates a *hold-to-activate* pattern: the task space mode is active while the
button is held, and joint space mode resumes when it is released.

4. Map input buttons for mode switching
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The commands listen for inputs named ``task_space`` and ``task_space_velocity``. Remap
these from your controller's physical buttons:

.. code-block:: yaml

   thrustmaster_right:
     ros__parameters:
       # ...
       remap:
         buttons:
           task_space:
             from: "r_btn_thumb_u_state"

   thrustmaster_left:
     ros__parameters:
       # ...
       remap:
         buttons:
           task_space_velocity:
             from: "l_btn_thumb_u_state"

5. Configure inputs for each mode
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Each control mode reads inputs by name. You can remap different physical inputs to the
same meaningful names for each input source, allowing both controllers to cooperate.

For example, the right Thrustmaster handles task-space angular inputs while the left
handles linear inputs:

.. code-block:: yaml

   thrustmaster_left:
     ros__parameters:
       remap:
         axes:
           linear:
             x:
               from: "l_ax_stick_y"
             y:
               from: "l_ax_stick_x"
             z:
               from: "l_ax_stick_twist"

   thrustmaster_right:
     ros__parameters:
       remap:
         axes:
           angular:
             x:
               from: "r_ax_stick_twist"
               invert: true
             y:
               from: "r_ax_stick_y"
             z:
               from: "r_ax_stick_x"
               invert: true

.. note::

   Nested YAML keys like ``linear: x: from: ...`` produce dotted input names. The
   example above provides ``linear.x``, ``linear.y``, and so on -- the same names
   that ``TwistControlMode`` requests.

Summary
-------

Complete multi-mode configuration structure:

.. code-block:: yaml

   teleop_arm:
     ros__parameters:
       control_modes:
         names:
           - joint_space_mode
           - task_space_mode
           - end_effector_mode

         joint_space_mode:
           display_name: "Joint Space"
           type: "joint_space_control_mode/JointSpaceControlMode"
           active: true
           controllers:
             - nova_arm_velocity_controller

         task_space_mode:
           display_name: "Task Space (IK)"
           type: "teleop_modular_twist/TwistControlMode"
           controllers:
             - "nova_twistmapper"
             - "nova_arm_position_controller"

         end_effector_mode:
           display_name: "End Effector"
           type: "joint_space_control_mode/JointSpaceControlMode"
           active: true
           controllers:
             - nova_end_effector_velocity_controller

       commands:
         names:
           - enter_task_space
           - enter_joint_space

         enter_task_space:
           on: "task_space/down"
           type: switch_control_mode
           activate:
             - "task_space_mode"
           deactivate:
             - "joint_space_mode"

         enter_joint_space:
           on: "task_space/up"
           type: switch_control_mode
           activate:
             - "joint_space_mode"
           deactivate:
             - "task_space_mode"

See also
--------

- :ref:`commands` -- full reference for commands and events
- :ref:`adding_a_lock`
- :ref:`writing_a_control_mode`
