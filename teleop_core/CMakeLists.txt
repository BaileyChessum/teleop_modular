cmake_minimum_required(VERSION 3.5)
project(teleop_core LANGUAGES CXX)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  control_msgs
  controller_manager_msgs
  geometry_msgs
  sensor_msgs
  pluginlib
  std_msgs
  std_srvs
  control_mode
  input_source
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
)

# -- Libraries

# Add plugin implementations here!
add_library(${PROJECT_NAME} SHARED
  src/commands/SwitchControlModeCommand.cpp
  src/commands/LogCommand.cpp
  src/commands/SetButtonCommand.cpp

  # These are also need to do unit tests
  src/inputs/InputCommonSpecializations.cpp
  src/inputs/Button.cpp
  src/inputs/Axis.cpp
  src/commands/Command.cpp
  src/control_modes/ControlModeManager.cpp
  src/inputs/InputManager.cpp
  src/events/EventCollection.cpp
  src/events/EventListenerQueue.cpp
  src/inputs/InputCollection.cpp

  # State
  src/inputs/state/StateManager.cpp
  src/inputs/state/StateCollection.cpp
  src/inputs/state/State.cpp

  src/input_sources/InputSourceHandle.cpp
  src/events/ButtonEvent.cpp
  src/events/Event.cpp
  src/events/EventManager.cpp

  src/input_sources/InputSourceManager.cpp
  src/commands/CommandManager.cpp
  src/commands/SetAxisCommand.cpp
  src/commands/IncrementAxisCommand.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)  # Require C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Stuff for plugins
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
install(FILES plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

# Test cases
#if(TR)
  find_package(ament_cmake_gtest REQUIRED)
#  find_package(ament_lint_auto REQUIRED)

  # Add tests
  ament_add_gtest(test_inputs
    test/test_inputs.cpp
    test/test_state.cpp
  )
  target_link_libraries(test_inputs
    ${PROJECT_NAME}
  )
  target_include_directories(test_inputs PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  set_tests_properties(test_inputs
    PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}"
  )

  # Enable linting
#  ament_lint_auto_find_test_dependencies()
#endif()

# -- Install

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
#  ${PROJECT_NAME}Targets
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()