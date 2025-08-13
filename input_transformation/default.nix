{ lib
, pkg-config
, buildRosPackage
, ament-cmake
, ament-cmake-gtest
, ament-lint-auto
, gtest
, rclcpp
, pluginlib
, teleop-modular-input-source
}:

buildRosPackage {
  name = "teleop-modular-input-transformation";
  buildType = "ament_cmake";

  src = builtins.path rec {
    name = "teleop-modular-input-transformation-source";
    path = ./.;
  };

  nativeBuildInputs = [
    ament-cmake
    pkg-config
    ament-cmake-gtest
    ament-lint-auto
    gtest
    teleop-modular-input-source
  ];

  buildInputs = [
    rclcpp
    pluginlib
  ];

  propagatedBuildInputs = [
  ];

  # Enable running tests during build
  doCheck = true;
}
