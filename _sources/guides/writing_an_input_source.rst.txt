.. _writing_an_input_source:

Writing an InputSource plugin
==============================

**Goal**: Create a custom plugin that provides inputs to ``teleop_modular`` from any source.

**Tutorial level**: Advanced

**Time**: 40 minutes

Background
----------

An ``InputSource`` is a pluginlib plugin that exposes a set of named axes and buttons
to the teleop system. You implement three methods:

- ``on_init()`` -- set up subscriptions, parameters, or threads
- ``export_axes()`` / ``export_buttons()`` -- declare the names and initial values of your inputs
- ``on_update()`` -- write the current values into the provided spans

``teleop_modular`` takes care of remapping, transformations, and thread safety -- your
implementation only needs to get values from your source and export them.

The ``JoyInputSource`` plugin (``teleop_modular_joy``) is a good reference implementation.

Prerequisites
-------------

- :ref:`writing_a_teleop_package`
- You should be comfortable writing ROS2 C++ packages.

Tasks
-----

1. Create the package
^^^^^^^^^^^^^^^^^^^^^^

Create a standard ROS2 ament_cmake package:

.. code-block:: bash

   ros2 pkg create --build-type ament_cmake my_input_source \
     --dependencies rclcpp input_source pluginlib

2. Declare dependencies
^^^^^^^^^^^^^^^^^^^^^^^^

In ``package.xml``, ensure you have:

.. code-block:: xml

   <depend>rclcpp</depend>
   <depend>input_source</depend>
   <build_depend>pluginlib</build_depend>

In ``CMakeLists.txt``:

.. code-block:: cmake

   find_package(rclcpp REQUIRED)
   find_package(input_source REQUIRED)
   find_package(pluginlib REQUIRED)

   add_library(my_input_source SHARED
     src/my_input_source.cpp
   )

   target_include_directories(my_input_source PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>
   )

   ament_target_dependencies(my_input_source
     rclcpp
     input_source
     pluginlib
   )

   pluginlib_export_plugin_description_file(input_source plugins.xml)

   install(TARGETS my_input_source
     EXPORT export_my_input_source
     ARCHIVE DESTINATION lib
     LIBRARY DESTINATION lib
     RUNTIME DESTINATION bin
   )

3. Write the header
^^^^^^^^^^^^^^^^^^^^

Create ``include/my_input_source/my_input_source.hpp``:

.. code-block:: cpp

   #pragma once

   #include <input_source/input_source.hpp>

   namespace my_input_source
   {

   class MyInputSource : public input_source::InputSource
   {
   public:
     input_source::return_type on_init() override;

     void export_buttons(input_source::InputDeclarationList<uint8_t> & declarations) override;
     void export_axes(input_source::InputDeclarationList<float> & declarations) override;

     input_source::return_type on_update(
       const rclcpp::Time & now,
       input_source::InputValueSpans values) override;

   private:
     // Your members here -- subscriptions, parameters, mutexes, etc.
   };

   }  // namespace my_input_source

4. Implement the source
^^^^^^^^^^^^^^^^^^^^^^^^

Create ``src/my_input_source.cpp``:

**on_init()**

Set up anything your input source needs -- subscriptions, parameter listeners, or threads:

.. code-block:: cpp

   #include "my_input_source/my_input_source.hpp"
   #include <pluginlib/class_list_macros.hpp>

   namespace my_input_source
   {

   input_source::return_type MyInputSource::on_init()
   {
     // get_node() provides your input source's ROS2 node
     subscription_ = get_node()->create_subscription<sensor_msgs::msg::Joy>(
       "/joy", rclcpp::QoS(10),
       [this](sensor_msgs::msg::Joy::SharedPtr msg) {
         // Call request_update() when new data arrives
         if (request_update(msg->header.stamp) == input_source::return_type::OK) {
           std::unique_lock lock{mutex_};
           latest_msg_ = msg;
         }
       });

     return input_source::return_type::OK;
   }

**export_buttons() and export_axes()**

Declare all the inputs your source provides. The order you add them here must
match the order you write values in ``on_update()``:

.. code-block:: cpp

   void MyInputSource::export_buttons(
     input_source::InputDeclarationList<uint8_t> & declarations)
   {
     declarations.add("button_a", 0);
     declarations.add("button_b", 0);
   }

   void MyInputSource::export_axes(
     input_source::InputDeclarationList<float> & declarations)
   {
     declarations.add("stick_x", 0.0f);
     declarations.add("stick_y", 0.0f);
   }

**on_update()**

Called after ``request_update()`` -- write the latest values into ``values``:

.. code-block:: cpp

   input_source::return_type MyInputSource::on_update(
     const rclcpp::Time & now,
     input_source::InputValueSpans values)
   {
     std::unique_lock lock{mutex_};
     if (!latest_msg_) {
       return input_source::return_type::OK;
     }

     // Write button values by index, matching the order in export_buttons()
     values.buttons[0] = latest_msg_->buttons[0] != 0;
     values.buttons[1] = latest_msg_->buttons[1] != 0;

     // Write axis values by index, matching the order in export_axes()
     values.axes[0] = latest_msg_->axes[0];
     values.axes[1] = latest_msg_->axes[1];

     return input_source::return_type::OK;
   }

**Plugin export macro**

At the bottom of the source file, register the class with pluginlib:

.. code-block:: cpp

   }  // namespace my_input_source

   #include <pluginlib/class_list_macros.hpp>

   PLUGINLIB_EXPORT_CLASS(my_input_source::MyInputSource, input_source::InputSource);

5. Register the plugin
^^^^^^^^^^^^^^^^^^^^^^^

Create ``plugins.xml`` in the package root:

.. code-block:: xml

   <library path="my_input_source">
     <class
       name="my_input_source/MyInputSource"
       type="my_input_source::MyInputSource"
       base_class_type="input_source::InputSource">
       <description>My custom input source.</description>
     </class>
   </library>

6. Use your input source in a teleop package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In your teleop parameter file, reference the plugin type and configure its parameters:

.. code-block:: yaml

   teleop_arm:
     ros__parameters:
       input_sources:
         names:
           - my_source

         my_source:
           type: "my_input_source/MyInputSource"

   # Remapping goes under the node named after the input source
   my_source:
     ros__parameters:
       remap:
         axes:
           linear.x:
             from: "stick_y"
         buttons:
           lock:
             from: "button_a"

.. note::

   The input source's parameters live on a node with the same name as the input
   source (``my_source`` in this example). You can declare and get parameters
   from this node inside ``on_init()`` using ``get_node()``.

   All of the standard ``remap:`` parameters are provided automatically for free
   by the framework -- you do not need to declare them yourself.

How request_update works
-------------------------

``request_update()`` is a thread-safe signal to the input manager that new data is
available. The input manager will then call ``on_update()`` from the update thread at
the appropriate time. This means:

- You can call ``request_update()`` from any thread (e.g. a subscription callback)
- ``on_update()`` is always called from the update thread, so you must protect shared
  data with a mutex
- If ``request_update()`` returns ``ERROR``, the update should be skipped for this cycle

See also
--------

- :ref:`remapping_and_transforming_inputs`
- :ref:`writing_a_control_mode`
- :ref:`writing_a_teleop_package`
