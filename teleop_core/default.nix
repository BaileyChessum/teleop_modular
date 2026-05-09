{ lib
, pkg-config
, buildRosPackage
, ament-cmake
, ament-cmake-gtest
, ament-lint-auto
, gtest
, rclcpp
, geometry-msgs
, lifecycle-msgs
, sensor-msgs
, pluginlib
, controller-manager-msgs
, control-msgs
, std-msgs
, std-srvs
, teleop-modular-control-mode
, teleop-modular-input-source
, rclcpp-lifecycle
}:

let
  packageArgs = {
    buildType = "ament_cmake";

    src = builtins.path rec {
      name = "teleop-modular-core-source";
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
      control-msgs
      rclcpp
      rclcpp-lifecycle
      std-srvs
      geometry-msgs
      lifecycle-msgs
      sensor-msgs
      pluginlib
      controller-manager-msgs
      control-msgs
      std-msgs
      teleop-modular-control-mode
      teleop-modular-input-source
    ];

    propagatedBuildInputs = [
      controller-manager-msgs
      control-msgs
      lifecycle-msgs
      std-srvs
      std-msgs
      teleop-modular-control-mode
      teleop-modular-input-source
      rclcpp-lifecycle
    ];

    # Added to debug a crazy segfault
    dontStrip = true;
    CMAKE_BUILD_TYPE = "Debug";
    cmakeFlags = [
      "-DCMAKE_BUILD_TYPE=Debug"
      "-DCMAKE_CXX_FLAGS=-g"
    ];
  };
in
let
  pkg = buildRosPackage (packageArgs // { name = "teleop-modular-core"; });
in
pkg // {
  passthru = pkg.passthru // {
    tests.unit = buildRosPackage (packageArgs // {
      name = "teleop-modular-core-tests";
      doCheck = true;
    });
  };
}
