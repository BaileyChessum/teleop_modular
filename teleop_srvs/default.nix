{ lib,
  buildRosPackage,
  ament-cmake,
  rosidl-default-generators,
}:

buildRosPackage {
  name = "teleop-srvs";
  buildType = "ament_cmake";

  src = builtins.path rec {
    name = "teleop-srvs-source";
    path = ./.;
    filter = lib.novaSourceFilter [ ] path;
  };

  nativeBuildInputs = [ ament-cmake rosidl-default-generators ];

  propagatedBuildInputs = [
  ];
}
