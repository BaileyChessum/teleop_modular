# Overlay to be used ontop of https://github.com/lopsided98/nix-ros-overlay
self: super:
{
  rosPackages = (super.rosPackages.appendDistroOverlay(
    rosSelf: rosSuper: {
      teleop-modular-control-mode = rosSuper.callPackage ./control_mode { };
      teleop-modular-input-source = rosSuper.callPackage ./input_source { };
      teleop-modular-core = rosSuper.callPackage ./teleop_core { };
      teleop-modular-twist = rosSuper.callPackage ./teleop_modular_twist { };
      teleop-modular-joy = rosSuper.callPackage ./teleop_modular_joy { };
      teleop-modular-msgs = rosSuper.callPackage ./teleop_msgs { };
      teleop-modular-srvs = rosSuper.callPackage ./teleop_srvs { };
      teleop-modular-node = rosSuper.callPackage ./teleop_node { };

      # Metapackage for teleop_modular, contains all of the above
      teleop-modular = rosSuper.callPackage ./teleop_modular { };
    }
  )) super.rosPackages;
}
