.. _getting_started:

Getting started
===============

This set of guides/tutorials will walk through writing complex teleop for a
`twist <https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html>`_-based robot using Teleop Modular. If you
don't have a robot, you should also be able to use
`turtlesim <https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/basics/ROS2-Turtlesim.html>`_.

Prerequisites
-------------

This documentation will assume a familiarity with fundamental ROS2 concepts.

If you're new to ROS2, follow these guides from the ROS2 Documentation to get started:

- `ROS2 - Installation <https://docs.ros.org/en/rolling/Installation.html>`_
- `ROS2 - Configuring environment <https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html>`_
- `ROS2 - Using colcon to build packages <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_
- `ROS2 - Launching nodes <https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html#id5>`_


Installation
------------

First, you'll need to make sure you have the teleop_modular packages added to your workspace. Currently, you will have 
to build the packages manually.

.. admonition:: Compatibility

   The CI workflow uses Ubuntu (latest) and a colcon workspace to build for:

   - ROS2 Rolling
   - ROS2 Kilted
   - ROS2 Jazzy
   - ROS2 Humble

   .. image:: https://github.com/BaileyChessum/teleop_modular/actions/workflows/ci.yml/badge.svg?branch=main
      :target: https://github.com/BaileyChessum/teleop_modular/actions/workflows/ci.yml?query=branch%3Amain
      :alt: CI (Rolling, Jazzy, and Humble)

   The package was primarily developed and tested on ROS2 Jazzy with the `nix <https://nixos.org/>`_ package manager using `lopsided98/nix-ros-overlay <https://github.com/lopsided98/nix-ros-overlay>`_ and `hacker1024/nix-ros-workspace <https://github.com/hacker1024/nix-ros-workspace>`_.

Building with colcon
--------------------

.. note::

   I don't yet serve a prebuilt underlay for you to use :(

For now, I recommend just cloning it into the workspace as a git submodule:

.. code-block:: sh

   # Using a colcon workspace (in a git repo!), add teleop_modular as a submodule
   cd workspace/src
   git submodule add https://github.com/BaileyChessum/teleop_modular.git teleop_modular

   # Set the revision to avoid unexpected updates to teleop_modular. Set the appropriate version.
   cd teleop_modular
   git checkout tags/v0.1.0

   # Commit your changes to the workspace
   cd ../../
   git add ./src/teleop_modular
   git commit -m "feat: added teleop_modular as a submodule"

   # You should now be able to build your workspace
   colcon build
   . install/setup.bash

Please `raise an issue <https://github.com/BaileyChessum/teleop_modular/issues/new>`_ or post in `discussions <https://github.com/BaileyChessum/teleop_modular/discussions/new?category=q-a>`_ if you have issues!

.. admonition:: Contributor Note

   I will work to make using colcon workspaces easier in the future. If this is your main workflow, I encourage you to
   contribute improvements to the installation process!

Building with nix
-----------------

If you're using `nix <https://nixos.org/>`_ with `lopsided98/nix-ros-overlay <https://github.com/lopsided98/nix-ros-overlay>`_, add `overlay.nix <../overlay.nix>`_ on top of it in your overlays.

If you don't already use nix, I wouldn't recommend using it to get started with Teleop Modular.

Running teleop_node
-------------------

Once added to your workspace, build and source it. You should be able to run ``teleop_node``:

.. code-block:: sh

   ros2 run teleop_node teleop_node

And your output might look like:

.. code-block:: console

   $ ros2 run teleop_node teleop_node

   [ERROR] [teleop_node]: control_modes.names was not set.
   [INFO] [teleop_node]: Control Modes:

   [INFO] [teleop_node]: Input Sources:

But until we configure ``teleop_node``, it won't do anything.

---

Follow the next guide to start writing your own teleop package with the framework:
- `Writing a teleop package <./writing_a_teleop_package.rst>`_
