{ lib
, buildRosPackage
, ament-cmake
, teleop-modular-core
, teleop-modular-control-mode
, teleop-modular-input-source
, pluginlib
, rclcpp
, fmt
, parameter-traits
, rclcpp-lifecycle
}:

buildRosPackage {
  name = "teleop-modular-node";

  src = builtins.path rec {
    name = "teleop-modular-node-source";
    path = ./.;
  };

  buildType = "ament_cmake";

  nativeBuildInputs = [
    ament-cmake
  ];

  buildInputs = [
    fmt
    parameter-traits
    rclcpp-lifecycle
    rclcpp
    ament-cmake
    pluginlib
    teleop-modular-control-mode
    teleop-modular-input-source
    teleop-modular-core
  ];

  propagatedBuildInputs = [
    teleop-modular-control-mode
    teleop-modular-input-source
    teleop-modular-core
  ];

  meta = {
    description = "ROS2 Node for teleop_modular.";
    license = with lib.licenses; [ asl20 ];
  };
}
