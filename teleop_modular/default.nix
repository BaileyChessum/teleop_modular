{ lib
, buildRosPackage
, ament-cmake
, teleop-modular-control-mode
, teleop-modular-input-source
, teleop-modular-core
, teleop-modular-joy
, teleop-modular-twist
, teleop-modular-node
, teleop-modular-msgs
, teleop-modular-srvs
, teleop-modular-python-utils
}:

buildRosPackage {
  name = "teleop-modular";

  src = builtins.path rec {
    name = "teleop-modular-source";
    path = ./.;
  };

  buildType = "ament_cmake";

  nativeBuildInputs = [
    ament-cmake
  ];

  buildInputs = [
    ament-cmake
  ];

  propagatedBuildInputs = [
    teleop-modular-control-mode
    teleop-modular-input-source
    teleop-modular-twist
    teleop-modular-joy
    teleop-modular-core
    teleop-modular-node
    teleop-modular-msgs
    teleop-modular-srvs
    teleop-modular-python-utils
  ];

  meta = {
    description = "Metapackage for teleop_modular, the modular framework for teleoperation in ROS2.";
    license = with lib.licenses; [ asl20 ];
  };
}
