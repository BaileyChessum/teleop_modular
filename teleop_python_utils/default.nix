{ lib
, writeShellApplication
, buildRosPackage
, rclpy
, teleop-modular-msgs
}:

buildRosPackage {
  name = "python-control";
  buildType = "ament_python";

  src = builtins.path rec {
    name = "python-control-source";
    path = ./.;
    filter = lib.novaSourceFilter [ ] path;
  };

  propagatedBuildInputs = [
    rclpy
    teleop-modular-msgs
  ];
}
