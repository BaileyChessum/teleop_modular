.. teleop_modular documentation master file, created by
   sphinx-quickstart on Tue Jul 29 17:13:35 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Teleop Modular Documentation
============================

Welcome to the documentation for the `teleop_modular <https://github.com/BaileyChessum/teleop_modular>`_ packages!

Teleop Modular is a general framework for multimodal teleoperation in ROS2 based on pluginlib.

.. toctree::
   :caption: Guides
   :hidden:

   guides/getting_started
   guides/writing_a_teleop_package
   guides/remapping_and_transforming_inputs
   guides/writing_a_control_mode

.. toctree::
   :maxdepth: 2
   :caption: About
   :hidden:

   about/input_source_remapping

Guides
------

To get started using Teleop Modular, please follow these guides:

* :ref:`getting_started`
* :ref:`writing_a_teleop_package`
* :ref:`remapping_and_transforming_inputs`
* :ref:`writing_a_control_mode`
* Using multiple control modes
* Adding a lock

These guides cover various ways to provide inputs:

* Complex input using state Commands
* Writing an `InputSource` plugin
* Providing inputs without an input source using services

These guides discuss advanced topics:

* Running multiple teleop_nodes

Architecture
------------


.. image:: images/diagrams/architecture_diagram_light.svg
    :width: 100%
    :align: center
    :alt: High level architecture diagram for a teleop_node

.. raw:: html

    <br/>

There are two main plugin types used in the teleop_modular framework:

* **Input Sources** provide input values
* **Control Modes** use *meaningfully* named inputs to create high level control commands to send to some control system.

.. hint::

    What does *meaningfully* named mean? Most teleop packages map inputs from specific sources directly to control messages:

    .. code-block:: cpp

        twist.linear.x = joy_msg[3];    //< Not meaningful, and tightly coupled to joy

    Instead, teleop_modular allows control modes to ask for with a name that describes what the input is actually used
    for:

    .. code-block:: cpp

        linear_.x = inputs.axes["linear.x"];  //< get input shared_ptr with meaningful name
        // ...
        twist.linear.x = *linear_.x;

Currently, two main input types are supported:

* **Axes**
  represent any number input.
* **Buttons**
  represent any boolean input.

Axes are used for most values in a control mode. However, buttons can be used in ways you might not expect.
A button can act as a "locked" value to E-stop the robot when true. Buttons can also invoke events, which run commands
that can modify the teleop system in some way, such as setting an input value, or activating/deactivating control modes.

The source code for the framework can be found in the `teleop_modular <https://github.com/BaileyChessum/teleop_modular>`_
GitHub repository.

UML Class Diagrams
~~~~~~~~~~~~~~~~~~

The following UML class diagrams describes the internal implementation in more detail for each package.


input_source
^^^^^^^^^^^^

.. image:: images/diagrams/uml/input_source_uml.svg
    :width: 100%
    :align: center
    :alt: input_source package UML class diagram


.. raw:: html

    <br/>


control_mode
^^^^^^^^^^^^

.. image:: images/diagrams/uml/control_mode_uml.svg
    :width: 76%
    :align: center
    :alt: control_mode package UML class diagram
