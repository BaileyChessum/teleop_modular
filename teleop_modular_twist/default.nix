{ lib
, pkg-config
, buildRosPackage
, ament-cmake
, ament-cmake-gtest
, ament-lint-auto
, gtest
, rclcpp
, geometry-msgs
, generate-parameter-library
, pluginlib
, std-msgs
, teleop-modular-control-mode
, std-srvs
}:

buildRosPackage {
  name = "teleop-modular-twist";
  buildType = "ament_cmake";

  src = builtins.path rec {
    name = "teleop-modular-twist-source";
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
    rclcpp
    std-msgs
    std-srvs
    generate-parameter-library
    geometry-msgs
    pluginlib
    teleop-modular-control-mode
  ];

  propagatedBuildInputs = [ geometry-msgs ];

  # Enable running tests during build
  doCheck = true;
}
