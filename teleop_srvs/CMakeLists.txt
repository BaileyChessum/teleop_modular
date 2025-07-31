cmake_minimum_required(VERSION 3.8)
project(teleop_srvs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Add definitions here
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ControlModeDetails.msg
  msg/InputSourceDetails.msg
  msg/PluginInstanceDetails.msg

  srv/IncrementAxis.srv
  srv/ListControlModes.srv
  srv/ListInputSources.srv
  srv/ListPluginInstances.srv
  srv/LoadPlugin.srv
  srv/SetAxis.srv
  srv/SetButton.srv
  srv/SwitchControlMode.srv
  srv/TriggerEvent.srv

  DEPENDENCIES std_msgs
)

ament_package()
