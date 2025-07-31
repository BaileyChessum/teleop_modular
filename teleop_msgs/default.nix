{ lib,
  buildRosPackage,
  ament-cmake,
  rosidl-default-generators,
  std-msgs
}:

buildRosPackage {
  name = "teleop-msgs";
  buildType = "ament_cmake";

  src = builtins.path rec {
    name = "teleop-msgs-source";
    path = ./.;
    filter = lib.novaSourceFilter [ ] path;
  };

  nativeBuildInputs = [ ament-cmake rosidl-default-generators std-msgs ];

  propagatedBuildInputs = [
    std-msgs
  ];
}
