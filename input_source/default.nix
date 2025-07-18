{ lib
, pkg-config
, buildRosPackage
, ament-cmake
, ament-cmake-gtest
, ament-lint-auto
, gtest
, rclcpp
, pluginlib
}:

buildRosPackage {
  name = "teleop-modular-input-source";
  buildType = "ament_cmake";

  src = builtins.path rec {
    name = "teleop-modular-input-source-source";
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

  propagatedBuildInputs = [ ];

  # Enable running tests during build
  doCheck = true;
}
