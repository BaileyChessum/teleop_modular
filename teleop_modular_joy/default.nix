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
, teleop-modular-input-source
, joy
}:

buildRosPackage {
  name = "teleop-modular-joy";
  buildType = "ament_cmake";

  src = builtins.path rec {
    name = "teleop-modular-joy-source";
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
    generate-parameter-library
    pluginlib
    teleop-modular-input-source
  ];

  propagatedBuildInputs = [ joy ];

  # Enable running tests during build
  doCheck = true;
}
