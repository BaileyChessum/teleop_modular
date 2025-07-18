{ lib
, pkg-config
, buildRosPackage
, ament-cmake
, ament-cmake-gtest
, ament-lint-auto
, gtest
, rclcpp
, pluginlib
, rclcpp-lifecycle
}:

buildRosPackage {
  name = "teleop-modular-control-mode";
  buildType = "ament_cmake";

  src = builtins.path rec {
    name = "teleop-modular-control-mode-source";
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
    pluginlib
  ];

  propagatedBuildInputs = [
    rclcpp-lifecycle
  ];

  # Enable running tests during build
  doCheck = true;
}
