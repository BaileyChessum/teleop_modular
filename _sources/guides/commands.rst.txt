.. _commands:

Commands and state inputs
=========================

**Goal**: Understand how commands and state inputs work, and use them for complex control logic.

**Tutorial level**: Intermediate

**Time**: 20 minutes

Background
----------

Commands are actions that fire in response to *events* -- typically button presses or
releases. They can change the teleop system's state in various ways: locking the robot,
switching control modes, adjusting a speed level, or logging a message.

*State inputs* are named buttons and axes whose values are managed by commands rather
than by an input source. They integrate with the rest of the input system: control modes
can read them, they can be remapped, and their transitions generate their own events
that can trigger further commands.

Prerequisites
-------------

- :ref:`writing_a_teleop_package`

Declaring commands
------------------

Commands are declared in the ``commands`` section of your teleop node's parameter file.
List the names of all commands in ``commands.names``, then configure each one:

.. code-block:: yaml

   teleop_arm:
     ros__parameters:
       commands:
         names:
           - my_command

         my_command:
           on: "some_button/down"
           type: set_button
           name: "my_state"
           value: true

Events
------

Each command must specify the event(s) that trigger it.

``on``
^^^^^^

Use ``on`` to trigger on a single event:

.. code-block:: yaml

   my_command:
     on: "lock/down"
     type: # ...

``on_any``
^^^^^^^^^^

Use ``on_any`` to trigger when *any* of a list of events fires:

.. code-block:: yaml

   my_command:
     on_any:
       - "lock/down"
       - "emergency/down"
     type: # ...

Event names
^^^^^^^^^^^

Event names have a suffix that determines what they listen to:

- ``"button_name/down"`` -- fires once when ``button_name`` transitions from false to true (pressed)
- ``"button_name/up"`` -- fires once when ``button_name`` transitions from true to false (released)

The button name can refer to any button available in the input system -- including state
buttons created by other commands. For example, ``"locked/down"`` fires when the
``locked`` state button becomes true.

Command types
-------------

``set_button``
^^^^^^^^^^^^^^

Sets a state button to a fixed value. Creates the state button if it does not yet exist.

.. code-block:: yaml

   lock:
     on: "lock/down"
     type: set_button
     name: "locked"    # Name of the state button to set
     value: true       # Value to set (true or false)

   unlock:
     on: "unlock/down"
     type: set_button
     name: "locked"
     value: false

``set_axis``
^^^^^^^^^^^^^

Sets a state axis to a fixed value. Creates the state axis if it does not yet exist.

.. code-block:: yaml

   set_speed_low:
     on: "speed_low/down"
     type: set_axis
     name: "speed"    # Name of the state axis to set
     value: 0.3       # Value to set (float)

``increment_axis``
^^^^^^^^^^^^^^^^^^

Increments a state axis by a fixed amount each time the command fires, up to a
maximum value.

.. code-block:: yaml

   increase_speed:
     on: "dpad_up/down"
     type: increment_axis
     name: "speed_level"    # Name of the state axis to increment
     by: 0.1                # Amount to increment each time the command fires
     until: 1.0             # Maximum value (increments stop here)
     log: true              # Whether to log the new value (default: true)

``switch_control_mode``
^^^^^^^^^^^^^^^^^^^^^^^

Activates and deactivates control modes. Use ``activate`` and ``deactivate`` to specify
which modes to change:

.. code-block:: yaml

   enter_task_space:
     on: "task_space/down"
     type: switch_control_mode
     activate:
       - "task_space_mode"
     deactivate:
       - "joint_space_mode"

``log``
^^^^^^^

Logs a message at the INFO level. Useful for tracking state changes.

.. code-block:: yaml

   log_locked:
     on: "locked/down"
     type: log
     message: "System locked"

State inputs
------------

A state input is created automatically the first time a command references it by name.
Once created, it behaves like any other named input in the system:

- Control modes can request it alongside hardware inputs
- It can be used in ``from:`` remap definitions
- Its transitions (e.g. ``"locked/down"``) generate events that other commands can react to

State buttons and axes persist across updates -- they retain their value until a command
changes them.

Example: a speed level controlled by buttons
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This creates a ``speed_level`` state axis that can be stepped up or down with D-pad
buttons, then used as a speed scaler in a control mode:

.. code-block:: yaml

   commands:
     names:
       - speed_up
       - speed_down

     speed_up:
       on: "DPAD_UP/down"
       type: increment_axis
       name: "speed_level"
       by: 0.25
       until: 1.0

     speed_down:
       on: "DPAD_DOWN/down"
       type: increment_axis
       name: "speed_level"
       by: -0.25
       until: 0.0

The ``speed_level`` state axis can then be used in the control mode's parameter file
wherever it accepts an input name.

Example: chaining commands via state events
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Commands can react to the events of state inputs created by other commands. This
creates a chain: when ``set_locked`` fires and sets ``locked`` to true, the
``"locked/down"`` event fires, which triggers ``log_locked``:

.. code-block:: yaml

   commands:
     names:
       - lock
       - unlock
       - log_locked
       - log_unlocked

     lock:
       on: "lock/down"
       type: set_button
       name: "locked"
       value: true

     unlock:
       on: "unlock/down"
       type: set_button
       name: "locked"
       value: false

     log_locked:
       on: "locked/down"    # Reacts to the *state button's* own event
       type: log
       message: "Locked"

     log_unlocked:
       on: "locked/up"
       type: log
       message: "Unlocked"

See also
--------

- :ref:`adding_a_lock`
- :ref:`using_multiple_control_modes`
- :ref:`remapping_and_transforming_inputs`
