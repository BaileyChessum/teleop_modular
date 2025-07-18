self: super:
{
  rosPackages = (super.rosPackages.appendDistroOverlay(
    rosSelf: rosSuper: {
      teleop-modular-control-mode = rosSuper.callPackage ./control_mode { };
      teleop-modular-input-source = rosSuper.callPackage ./input_source { };
      teleop-modular = rosSuper.callPackage ./teleop_modular { };
      teleop-modular-twist = rosSuper.callPackage ./teleop_modular_twist { };
      teleop-modular-joy = rosSuper.callPackage ./teleop_modular_joy { };
    }
  )) super.rosPackages;
}
