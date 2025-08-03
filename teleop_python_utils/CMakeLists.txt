cmake_minimum_required(VERSION 3.8)
project(teleop_python_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(teleop_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()