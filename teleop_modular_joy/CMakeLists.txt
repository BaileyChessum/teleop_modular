cmake_minimum_required(VERSION 3.8)
project(teleop_modular_joy)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  pluginlib
  rclcpp
  std_msgs
  geometry_msgs
  joy
  input_source
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
)

# -- Libraries

generate_parameter_library(
  joy_input_source_parameters
  src/joy_input_source_parameters.yaml
)

add_library(${PROJECT_NAME}
  src/teleop_modular_joy.cpp
)

add_library(teleop_modular_joy::teleop_modular_joy ALIAS teleop_modular_joy)
target_compile_features(teleop_modular_joy PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(teleop_modular_joy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Link the generate_parameter_library
target_link_libraries(${PROJECT_NAME}
  joy_input_source_parameters
)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "TELEOP_MODULAR_JOY_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(teleop_modular plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  joy_input_source_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  teleop_modular_joy
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
