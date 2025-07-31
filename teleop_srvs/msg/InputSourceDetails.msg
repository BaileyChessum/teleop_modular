
name: string    # The name of the input source
type: string    # The pluginlib type name
ok: bool        # Whether the plugin was loaded ok
status: string  # Any additional information on the status of the plugin. Populated with error message on failure

string[] exported_buttons   # Names of exported buttons after remapping
string[] exported_axes      # Names of exported axes after remapping
