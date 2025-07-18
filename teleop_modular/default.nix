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
, generate-parameter-library
, pluginlib
, controller-manager-msgs
, control-msgs
, std-msgs
, std-srvs
, teleop-modular-control-mode
, teleop-modular-input-source
}:

buildRosPackage {
  name = "teleop-modular";
  buildType = "ament_cmake";

  src = builtins.path rec {
    name = "teleop-modular-source";
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
    std-srvs
    generate-parameter-library
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
  ];

  # Enable running tests during build
  doCheck = true;
}
