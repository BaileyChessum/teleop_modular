{ lib
, writeShellApplication
, buildRosPackage
, rclpy
, teleop-modular-msgs
, ament-cmake-python
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
  ];

  propagatedBuildInputs = [
    rclpy
    teleop-modular-msgs
  ];

  doCheck = true;
}
