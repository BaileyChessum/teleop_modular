{ lib
, writeShellApplication
, buildRosPackage
, rclpy
, teleop-modular-msgs
, ament-cmake-python
, python3Packages
}:

buildRosPackage {
  name = "teleop-python-utils";
  buildType = "ament_cmake_python";

  src = builtins.path rec {
    name = "teleop-python-utils-source";
    path = ./.;
  };

  nativeBuildInputs = [
    ament-cmake-python
  ];

  buildInputs = [
    rclpy
    python3Packages.pytest
  ];

  propagatedBuildInputs = [
    rclpy
    teleop-modular-msgs
    python3Packages.pytest
  ];

  doCheck = true;
}
