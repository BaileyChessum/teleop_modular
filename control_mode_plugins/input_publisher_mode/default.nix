# You only need to use this file if you use nix to build your packages.
# If you don't, feel free to delete this.

{ lib
, pkg-config
, buildRosPackage
, ament-cmake
, ament-cmake-gtest
, ament-lint-auto
, gtest
, rclcpp
, pluginlib
, teleop-modular-control-mode
, teleop-modular-msgs
, rclcpp-lifecycle
}:

buildRosPackage {
  name = "input-publisher-mode";
  buildType = "ament_cmake";

  src = builtins.path rec {
    name = "input-publisher-mode-source";
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

  propagatedBuildInputs = [
    # Add message packages here
    teleop-modular-msgs
  ];

  # Enable running tests during build?
  # doCheck = true;
}
