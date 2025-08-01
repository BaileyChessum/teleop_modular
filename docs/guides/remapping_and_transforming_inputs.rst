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

.. note::

   This guide is yet to be written. All the tools you need to figure out how to solve the given problem should be available
   in :ref:`input_source_remapping`_ if you want to give it a crack early.

See also:
---------

- :ref:`writing_a_control_mode`
