{ pkgs }:

with pkgs;

{
  teleop-modular-input-publisher-mode = callPackage ./input_publisher_mode { };
  teleop-modular-twist = callPackage ./teleop_modular_twist { };
}
