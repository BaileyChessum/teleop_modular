{ lib
, pkg-config
, buildRosPackage
, ament-cmake
, ament-cmake-gtest
, ament-lint-auto
, gtest
, rclcpp
, geometry-msgs
, sensor-msgs
, pluginlib
, controller-manager-msgs
, control-msgs
, std-msgs
, std-srvs
, teleop-modular-control-mode
, teleop-modular-input-source
, rclcpp-lifecycle
, teleop-modular-msgs
}:

buildRosPackage {
  name = "teleop-modular-core";
  buildType = "ament_cmake";



  src = builtins.path rec {
    name = "teleop-modular-core-source";
    path = ./.;
  };

  nativeBuildInputs = [
    ament-cmake
    pkg-config
    ament-cmake-gtest
    ament-lint-auto
    gtest
  ];

  buildInputs = [
    control-msgs
    rclcpp
    rclcpp-lifecycle
    std-srvs
    geometry-msgs
    sensor-msgs
    pluginlib
    controller-manager-msgs
    control-msgs
    std-msgs
    teleop-modular-control-mode
    teleop-modular-input-source
  ];

  propagatedBuildInputs = [ 
    controller-manager-msgs
    control-msgs 
    std-srvs
    std-msgs
    teleop-modular-control-mode
    teleop-modular-input-source
    teleop-modular-msgs
    rclcpp-lifecycle
  ];

  # Enable running tests during build
  doCheck = true;

  # Added to debug a crazy segfault
  dontStrip = true;
  CMAKE_BUILD_TYPE = "Debug";
  cmakeFlags = [
    "-DCMAKE_BUILD_TYPE=Debug"
    "-DCMAKE_CXX_FLAGS=-g"
  ];
}
