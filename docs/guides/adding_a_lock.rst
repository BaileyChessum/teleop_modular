.. _adding_a_lock:

Adding a lock
=============

**Goal**: Add a safety lock that halts robot motion when active.

**Tutorial level**: Intermediate

**Time**: 15 minutes

Background
----------

A *lock* is a safety mechanism that prevents the robot from moving when active --
similar to an E-stop or deadman switch. In ``teleop_modular``, locking is implemented
using a *state input*: a button whose value is controlled by commands rather than
directly from a hardware input source.

Control modes check this state button on every update. When it is ``true``, the control
mode stops publishing motion commands and publishes a halt message instead.

Prerequisites
-------------

You should be familiar with the basics of configuring a teleop package:

- :ref:`writing_a_teleop_package`

Tasks
-----

1. Add lock and unlock commands
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

State inputs are created automatically when a command first references their name. To
create a ``locked`` state button, declare two ``set_button`` commands in your teleop
parameter file:

.. code-block:: yaml

   teleop_arm:
     ros__parameters:
       # ...
       commands:
         names:
           - lock
           - unlock

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

- ``type: set_button`` -- sets the named state button to the given value when the command fires
- ``on: "lock/down"`` -- fires once when the ``lock`` input button is pressed (transitions from false to true)
- ``on: "unlock/down"`` -- fires once when the ``unlock`` input button is pressed

2. Map physical buttons to ``lock`` and ``unlock``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The commands listen to inputs named ``lock`` and ``unlock``. Remap these from your
controller's physical buttons:

.. code-block:: yaml

   joy_input_source:
     ros__parameters:
       # ...
       remap:
         buttons:
           lock:
             from: "BACK"
           unlock:
             from: "START"

3. Confirm your control mode respects the lock
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Built-in control modes like ``teleop_modular_twist/TwistControlMode`` automatically
check the ``locked`` state button and halt when it is true. If you are writing a
custom control mode, check ``is_locked()`` in your ``on_update()`` and publish a halt
message:

.. code-block:: cpp

   return_type MyControlMode::on_update(
     const rclcpp::Time & now, const rclcpp::Duration & period) override
   {
     if (is_locked()) {
       publish_halt_message(now);
       return return_type::OK;
     }
     // ... normal update logic ...
   }

4. Add log messages when the lock state changes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is good practice to log whenever the system is locked or unlocked. Add ``log``
commands that trigger on the *state button's own events*:

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
       on: "locked/down"
       type: log
       message: "Locked"

     log_unlocked:
       on: "locked/up"
       type: log
       message: "Unlocked"

Notice that ``log_locked`` fires on ``"locked/down"`` -- the event for the state
button ``locked`` itself becoming true -- not the physical ``lock`` button. This means
the log fires whenever the state changes, regardless of what caused it.

Summary
-------

Complete lock configuration using a gamepad:

.. code-block:: yaml

   teleop_arm:
     ros__parameters:
       # ...
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
           on: "locked/down"
           type: log
           message: "Locked"

         log_unlocked:
           on: "locked/up"
           type: log
           message: "Unlocked"

   joy_input_source:
     ros__parameters:
       # ...
       remap:
         buttons:
           lock:
             from: "BACK"
           unlock:
             from: "START"

See also
--------

- :ref:`commands` -- full reference for all command types and events
- :ref:`writing_a_control_mode`
