.. _remapping_and_transforming_inputs:

Remapping and transforming inputs
=================================

**Goal**: Learn how to make control schemes with input source remapping.

**Tutorial level**: Intermediate

**Time**: 20 minutes

Background
----------

Teleop Modular adds a set of remap parameters to every input source
implementation. They allow us to:

* Rename inputs
* Create axes from buttons
* Create buttons from axes
* Transform input values in various ways, such as:

   * Inverting axes and buttons
   * Linearly mapping an input range of axis values to an output range
   * Clamping axes
   * Raising axes to a power

This guide walks through a practical example. For the full parameter reference, see
:ref:`input_source_remapping`.

Prerequisites
-------------

You should be familiar with setting up a teleop package:

* :ref:`writing_a_teleop_package`

Tasks
-----

1. Define names for your input source's outputs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Input sources export their raw inputs under the names defined in their configuration.
For ``JoyInputSource``, this means setting ``axis_definitions`` and ``button_definitions``
to match the physical layout of your controller:

.. code-block:: yaml

   joy_input_source:
     ros__parameters:
       topic: "/joy"
       axis_definitions: [
         "left_stick_x",
         "left_stick_y",
         "right_stick_x",
         "right_stick_y",
         "right_trigger"
       ]
       button_definitions: [
         "dpad_left",
         "dpad_right",
         "start",
         "share"
       ]

These names are what you will use in the ``remap:`` parameters below.

2. Rename inputs
^^^^^^^^^^^^^^^^

Your control modes request inputs by the names they expect (e.g. ``linear.x``, ``speed``).
Use the ``remap:`` parameters to map from the exported source names to the names your
control mode uses:

.. code-block:: yaml

   joy_input_source:
     ros__parameters:
       # ...
       remap:
         axes:
           linear.x:
             from: "left_stick_y"
           linear.y:
             from: "left_stick_x"
           angular.z:
             from: "right_stick_x"
         buttons:
           lock:
             from: "start"
           unlock:
             from: "share"

Any input that is remapped to a new name is only available under that new name -- the
original name is no longer exposed.

.. note::

   If you want to apply a transformation without renaming, still specify ``from`` with
   the same name as the output.

3. Transform axis values
^^^^^^^^^^^^^^^^^^^^^^^^

Invert an axis when a joystick axis is backwards relative to what your control mode expects:

.. code-block:: yaml

   remap:
     axes:
       linear.x:
         from: "left_stick_y"
         invert: true

Remap a trigger from its native ``[-1.0, 1.0]`` range to a ``[0.0, 1.0]`` speed range:

.. code-block:: yaml

   remap:
     axes:
       speed:
         from: "right_trigger"
         range:
           in: [-1.0, 1.0]
           out: [0.0, 1.0]

Apply an exponential curve to reduce sensitivity near the centre of a joystick:

.. code-block:: yaml

   remap:
     axes:
       angular.z:
         from: "right_stick_x"
         power: 2.0

4. Create a button from an axis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can derive a button from an axis with an optional threshold. The button is active
(``true``) when the axis value is below the threshold (default ``0.0``):

.. code-block:: yaml

   remap:
     buttons:
       braking:
         from_axis:
           name: "right_trigger"
           threshold: 0.5  # button is true when axis < 0.5

5. Create an axis from buttons
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can synthesise a virtual axis from a pair of buttons -- useful for D-pad inputs:

.. code-block:: yaml

   remap:
     axes:
       linear.y:
         from_buttons:
           negative: "dpad_left"
           positive: "dpad_right"
           default_value: 0.0

When the positive button is pressed the axis outputs ``1.0``, when the negative button
is pressed it outputs ``-1.0``, and when neither is pressed it outputs ``default_value``.

Summary
-------

The complete example parameter file combining all of the above:

.. code-block:: yaml

   joy_input_source:
     ros__parameters:
       topic: "/joy"
       axis_definitions: [
         "left_stick_x",
         "left_stick_y",
         "right_stick_x",
         "right_stick_y",
         "right_trigger"
       ]
       button_definitions: [
         "dpad_left",
         "dpad_right",
         "start",
         "share"
       ]
       remap:
         axes:
           linear.x:
             from: "left_stick_y"
             invert: true
           linear.y:
             from_buttons:
               negative: "dpad_left"
               positive: "dpad_right"
               default_value: 0.0
           angular.z:
             from: "right_stick_x"
             power: 2.0
           speed:
             from: "right_trigger"
             range:
               in: [-1.0, 1.0]
               out: [0.0, 1.0]
         buttons:
           lock:
             from: "start"
           unlock:
             from: "share"

See also
--------

- :ref:`input_source_remapping` -- full parameter reference for all remapping options
- :ref:`writing_a_control_mode`
