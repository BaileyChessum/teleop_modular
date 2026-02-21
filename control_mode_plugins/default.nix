{ pkgs }:

with pkgs;

{
  teleop-modular-input-publisher-mode = callPackage ./input_publisher_mode { };
  teleop-modular-locked-publisher = callPackage ./locked_publisher { };
  teleop-modular-twist = callPackage ./teleop_modular_twist { };
}
