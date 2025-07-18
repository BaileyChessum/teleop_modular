{
  rosPackages = pkgs: with pkgs; {
    teleop-modular-control-mode = callPackage ./control_mode { };
    teleop-modular = callPackage ./teleop_modular { inherit teleop-modular-control-mode; };
    teleop-modular-twist = callPackage ./teleop_modular_twist { };
    teleop-modular-joy = callPackage ./teleop_modular_joy { };
  };

  shellAliases = {

  };
}
