{ lib
, writeShellApplication
, buildRosPackage
, rclpy
, teleop-modular-msgs
}:

buildRosPackage {
  name = "teleop-python-utils";
  buildType = "ament_python";

  src = builtins.path rec {
    name = "teleop-python-utils-source";
    path = ./.;
  };

  propagatedBuildInputs = [
    rclpy
    teleop-modular-msgs
  ];

  doCheck = true;
}
