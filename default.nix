{
  rosPackages = pkgs: with pkgs; {
    teleop-modular = callPackage ./teleop_modular { };
    teleop-modular-twist = callPackage ./teleop_modular_twist { };
#    teleop-twist-control-mode = callPackage ./twist_control_mode { };
#    teleop-joy-input-source = callPackage ./joy_input_source { };
  };

  shellAliases = {

  };
}
