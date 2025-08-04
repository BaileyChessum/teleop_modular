.. _writing_a_control_mode:

Writing a ``ControlMode`` plugin
================================

**Goal**: Create a custom ``ControlMode`` plugin using C++.

**Tutorial level**: Intermediate

**Time**: 20 minutes

Background
----------

The purpose of ``teleop_modular`` is not to be a fancy generic teleop package, but to provide infrastructure you can use
to create your own teleop package, with specialized control modes.

It should be easy to create your own control mode plugins to add to your teleop package. This guide will show you how.

To create the package, we will be using `cookiecutter <https://www.cookiecutter.io/>`_ and
`control_mode_template <https://github.com/BaileyChessum/control_mode_template>`_.

Prerequisites
-------------

To test your custom control mode, you should probably set up a simple teleop package to run it:

- :ref:`getting_started`
- :ref:`writing_a_teleop_package`

Tasks
-----

1. Install cookiecutter
^^^^^^^^^^^^^^^^^^^^^^^

Start by installing cookiecutter.

- `Installation - cookiecutter documentation <https://cookiecutter.readthedocs.io/en/1.7.3/installation.html>`_

It is a python package, so you can get it through vendors like pip:

.. code-block:: console

   $ pip install --user cookiecutter

On Debian / Ubuntu, you can get it from:

.. code-block:: console

   $ sudo apt-get install cookiecutter

Or if you have nix, open up a shell with cookiecutter:

.. code-block:: console

   $ nix-shell -p cookiecutter

2. Create the package
^^^^^^^^^^^^^^^^^^^^^

Open a terminal and navigate into your workspace src directory:

.. code-block:: console

   $ cd path/to/ws/src

Now, create the package with `control_mode_template <https://github.com/BaileyChessum/control_mode_template>`_:

.. code-block:: console

   $ cookiecutter https://github.com/BaileyChessum/control_mode_template.git

You should then be prompted to enter the name of the package, the control mode, and some extra info to generate the
package:

.. code-block:: console

   [1/6] project_name (example_control_mode): example_control_mode
   [2/6] control_mode_class_name (ExampleControlMode): ExampleControlMode
   [3/6] author_name (TODO: Author Name): Bailey Chessum
   [4/6] author_email (author@example.com): bailey.chessum1@gmail.com
   [5/6] license (TODO: License Definition): Apache-2.0
   [6/6] description (A custom teleop_modular control mode.): An example teleop_modular control mode.

.. note::

   Make sure to use snake_case for ``project_name``, and UpperCamelCase for ``control_mode_class_name``.

You should now have a new package for your control mode. The directory structure might look something like:

.. code-block:: none

   ws/
   └── src/
       └── project_name/
           ├── include/
           │   └── project_name/
           │       ├── project_name.hpp
           │       └── visibility_control.h
           ├── src/
           │   └── project_name.cpp
           ├── CMakeLists.txt
           ├── README.md
           ├── package.xml
           └── plugins.xml

You should be able to build it as part of your workspace, and run it with your teleop package.

Check ``plugins.xml`` or the title of the generated ``README.md`` to find the string to use for ``.type`` in your teleop_node
config. It should be ``project_name/control_mode_class_name``. A simple example parameter file can also be found at the
bottom of ``README.md``.

- See :ref:`writing_a_teleop_package` for help setting up and running a control mode with teleop_modular.

Open up the project in your favourite IDE with the necessary dependencies available in your shell and load
``CMakeLists.txt``. Then, let's get started editing the control mode.

You'll find lots of helpful ``// TODO:`` comments in the codebase to help point you in the right direction as to what
needs to be implemented.

3. Add dependencies
^^^^^^^^^^^^^^^^^^^

You'll likely need to have your control mode publish some ROS2 message to your control system. You will need to add the
package the message type comes from as a dependency to your package.

open up ``package.xml`` and add your dependency:

.. code-block:: xml

  <depend>my_interfaces</depend>

Then, open up ``CMakeLists.txt`` and add your dependency to ``set(THIS_PACKAGE_INCLUDE_DEPENDS``:

.. code-block:: cmake

   cmake_minimum_required(VERSION 3.8)
   project(example_control_mode)

   set(THIS_PACKAGE_INCLUDE_DEPENDS
     rclcpp
     rclcpp_lifecycle
     pluginlib
     control_mode

     # Add your dependency here:
     my_interfaces
   )

   # ...

4. Add a publisher
^^^^^^^^^^^^^^^^^^

Now that you have any packages you need for message types or otherwise, you can add a publisher to ``project_name.hpp``
and ``project_name.cpp``.

.. note::

   Your control mode doesn't necessarily need to publish messages to ROS2. You could have a control mode that
   calls an Action, or activates an external LifecycleNode, or anything at all! You have free will.

At the top of your ``.hpp``, import your message type:

.. code-block:: cpp

   // project_name.hpp
   #ifndef PROJECT_NAME__PROJECT_NAME_HPP_
   #define PROJECT_NAME__PROJECT_NAME_HPP_

   #include <rclcpp/time.hpp>
   #include <string>
   #include "control_mode/control_mode.hpp"
   #include "project_name/visibility_control.h"

   // Include your message type:
   #include "my_interfaces/msg/example_message.hpp"

   namespace project_name
   {
   // ...

Then, add the appropriate publisher as a member variable in ``project_name.hpp``. You'll find a relevant ``// TODO:``:

.. code-block:: cpp

   // project_name.hpp
   // ...
   private:
     /// Helper struct to hold parameters used by the control mode.
     struct Params {
       /// The topic name to send messages to.
       std::string topic = "";
       /// The ROS2 topic Quality of Service value to use in publisher_.
       int qos = 10;
     };

     /// Stores current parameter values
     Params params_;

     // TODO: Set an appropriate message type for the publisher, then uncomment its declaration/usages

     // Uncomment the publisher, and insert the correct message type here:
     rclcpp::Publisher<my_interfaces::msg::ExampleMessage>::SharedPtr publisher_;

   // ...

Then, create the publisher in ``on_configure()`` in the ``.cpp`` file. You'll again find a relevant ``// TODO:``:

.. code-block:: cpp

   // project_name.cpp
   // ...
   CallbackReturn ExampleControlMode::on_configure(const State &)
   {
     auto node = get_node();
     const auto logger = get_node()->get_logger();

     // Use this callback method to get any parameters for your control mode!
     params_ = Params();
     node->get_parameter<std::string>("topic", params_.topic);
     node->get_parameter<int>("qos", params_.qos);

     // Create the publishers based on the params we just got
     if (params_.topic.empty()) {
       // You've probably made a mistake if the topic isn't set!
       RCLCPP_ERROR(logger, "The \"topic\" parameter must be set to a valid topic name!");
       return CallbackReturn::ERROR;
     }

     // TODO: Set an appropriate message type for the publisher, then uncomment its declaration/usages

     // Uncomment the publisher, and insert the correct message type here:
     publisher_ = get_node()->create_publisher<my_interfaces::msg::ExampleMessage>(params_.topic, params_.qos);

     return CallbackReturn::SUCCESS;
   }
   // ...

5. Capture inputs
^^^^^^^^^^^^^^^^^

To put values into your message during ``update()``, you'll need to have stored references to some inputs to provide the
values to use in the message.

For all the inputs you need, add member variables to store them. This could be an ``Axis::SharedPtr`` for floating point
values, ``Button::SharedPtr`` for true/false values, or some other structure that contains these pointers, such as a
``std::vector<Axis::SharedPtr>``:

.. code-block:: cpp

   // project_name.hpp
     // ...

     // TODO: Add shared pointers for any buttons/axes you need here, then set them in on_configure_inputs().

     // You can hold references to inputs like this, and set their values in on_configure_inputs:
     /// Input from 0 to 1 that directly scales the output speed.
     Axis::SharedPtr speed_;

     // Add any inputs you want here:
     Axis::SharedPtr some_axis_;


     // ...
   }

Then, in ``on_configure_inputs()`` in your ``.cpp`` file, assign your input shared pointers:

.. code-block:: cpp

   // project_name.cpp
   // ...
   void ExampleControlMode::on_configure_inputs(Inputs inputs)
   {
     // This method is always run after on_configure(),
     // so you can assume that you already have any necessary parameters

     // Capture inputs like this:
     speed_ = inputs.axes["speed"];

     // TODO: Add Axis::SharedPtr and/or Button::SharedPtr member variables, then assign them here.

     // Assign your input shared pointers here:
     some_axis_ = inputs.axes["some_axis_name"];
     some_button_ = inputs.buttons["some_button_name"];
   }
   // ...

.. note::

  You can name the inputs whatever you want. Just avoid ``/``, spaces, and empty strings.

6. Publish messages
^^^^^^^^^^^^^^^^^^^

Now, you can use the inputs you captures to create messages to publish.

In ``on_update()`` in your ``.cpp`` file, create a message, assign the message values using input values, then publish it:

.. code-block:: cpp

   // project_name.cpp
   // ...
   return_type ExampleControlMode::on_update(const rclcpp::Time & now, const rclcpp::Duration & period)
   {
     // Don't move when locked
     if (is_locked()) {
       publish_halt_message(now);
       return return_type::OK;
     }

     // Get input values either with input_->value() or by referencing and implicitly casting *input_
     const float speed = std::max(speed_->value(), 0.0f);

     // TODO: Construct and send a message using values from inputs

     // Uncomment this and put in the correct message type:
     auto msg = std::make_unique<my_interfaces::msg::ExampleMessage>();

     // Set values in your message type:
     msg->some_value = *some_axis_ * speed;
     // Alternatively, you can write:
     // msg->some_value = some_axis_->value() * speed;

     // If your message has a header, you can do this:
     msg->header.stamp = now;

     // Uncomment this:
     publisher_->publish(std::move(msg));

     return return_type::OK;
   }
   // ...

It's also good practice to have your control mode publish some kind of 'halt' message whenever the control mode is
locked. If applicable, implement ``publish_halt_message()`` similar to above:

.. code-block:: cpp

   // project_name.cpp
   // ...
   void {{cookiecutter.control_mode_class_name}}::publish_halt_message(const rclcpp::Time & now) const
   {
     // TODO: Implement for your message type, or remove the method if it is not appropriate for the use case.

     // Uncomment this and put in the correct message type:
     auto msg = std::make_unique<TODO>();

     // If your message has a header, you can do this:
     msg->header.stamp = now;

     // Uncomment this:
     publisher_->publish(std::move(msg));
   }
   // ...

7. Run your control mode
^^^^^^^^^^^^^^^^^^^^^^^^

Now that you have you control mode written, add it to your parameter file for your teleop node:

.. code-block:: yaml

   # teleop.yaml, or whatever you named it
   teleop_node:  # or whatever you renamed it
     ros__parameters:
       # ...

       # Change this to use your new control mode
       control_modes:
         names: [
           # Give your mode a name:
           "custom_control_mode"
         ]

         custom_control_mode:
           # You can find the type name to use in the autogenerated README.md
           type: "project_name/ExampleControlMode"

   # ...

   # Add config for the control mode's node
   custom_control_mode:
     ros__parameters:
       # Set the appropriate topic here!
       topic: "/example_topic"

Then, rebuild your workspace, source it, and try run your teleop package:

.. code-block:: console

   $ ros2 launch teleop_example teleop.launch.py

Your control mode plugin should be listed:

.. parsed-literal::

   $ ros2 launch teleop_arm teleop.launch.py teleop_params:=/home/.../teleop_example/params/teleop.yaml
   [INFO] [launch]: All log files can be found below /home/nova/.ros/log/2025-07-21-01-35-16-545322-nixos-2213788
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [teleop_node-1]: process started with pid [2213791]
   [teleop_node-1] [INFO] [teleop_example] Control Modes:
   [teleop_node-1] 	- Custom Control Mode	: project_name/ExampleControlMode
   [teleop_node-1]
   [teleop_node-1] [INFO] [teleop_example] Input Sources:
   [teleop_node-1]
   [teleop_node-1] [INFO] [teleop_example] Custom Control Mode activated

Enjoy writing control modes!

If you have any issues, please post in
`Discussions <https://github.com/BaileyChessum/teleop_modular/discussions/new?category=q-a>`_, and I will try to help!
