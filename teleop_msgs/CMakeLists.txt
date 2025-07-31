cmake_minimum_required(VERSION 3.8)
project(teleop_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Add definitions here
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Inputs.msg
  msg/InputNames.msg
  msg/InputValues.msg
  msg/ControlModeState.msg

  DEPENDENCIES std_msgs
)

ament_package()
