string name         # control mode name
string type         # the control mode class name, e.g. teleop_modular_twist/TwistControlMode
string state        # control mode state: unconfigured, inactive, active, or finalized
bool keep_alive     # Whether the control mode should be able to deactivated with "*"

# TODO: Capture information on the inputs a control mode consumes, and add a field for them here
# (information likely to be captured when implementing input name remapping on control modes).