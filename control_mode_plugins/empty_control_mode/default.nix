{ lib
, pkg-config
, buildRosPackage
, ament-cmake
, ament-cmake-gtest
, ament-lint-auto
, gtest
, rclcpp
, rclcpp-lifecycle
, pluginlib
, teleop-modular-control-mode
}:

buildRosPackage {
  name = "empty-control-mode";
  buildType = "ament_cmake";

  src = builtins.path rec {
    name = "empty-control-mode-source";
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
    rclcpp-lifecycle
    pluginlib
    teleop-modular-control-mode
  ];

  # Enable running tests during build?
  # doCheck = true;
}
