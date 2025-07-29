.. teleop_modular documentation master file, created by
   sphinx-quickstart on Tue Jul 29 17:13:35 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Teleop Modular Documentation
============================

Welcome to the documentation for the `teleop_modular <https://github.com/BaileyChessum/teleop_modular>`_ packages!

Teleop Modular is a general framework for multimodal teleoperation in ROS2 based on pluginlib.

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

