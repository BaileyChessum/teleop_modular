{
  rosPackages = pkgs: with pkgs; {
    teleop-modular-control-mode = callPackage ./control_mode { };
    teleop-modular-input-source = callPackage ./input_source { };
    teleop-modular = callPackage ./teleop_modular { };
    teleop-modular-twist = callPackage ./teleop_modular_twist { };
    teleop-modular-joy = callPackage ./teleop_modular_joy { };
  };

  shellAliases = {

  };
}
