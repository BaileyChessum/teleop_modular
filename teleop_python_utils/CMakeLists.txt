cmake_minimum_required(VERSION 3.8)
project(teleop_python_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(teleop_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Add pytest tests
#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
#
#  # Add a test using pytest
#  ament_add_pytest_test(test_teleop_python_utils
#    test/test_init.py
#    APPEND_ENV PYTHONPATH=$<TARGET_FILE_DIR:${PROJECT_NAME}>
#  )
#endif()


ament_package()