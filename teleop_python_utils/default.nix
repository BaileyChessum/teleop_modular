{ lib
, writeShellApplication
, buildRosPackage
, rclpy
, teleop-modular-msgs
, ament-cmake-python
, python3Packages
, ament-lint-auto
, ament-cmake-pytest
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
    ament-lint-auto
    ament-cmake-pytest
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

  # ament_add_pytest_test is currently not working with the ament-cmake-pytest dependency?
  doCheck = false;
}
