# Used to describe the status of input sources, commands, or other plugin types

name: string    # The name of the instance of the plugin
type: string    # The pluginlib type name
ok: bool        # Whether the plugin was loaded ok
status: string  # Any additional information on the status of the plugin. Populated with error message on failure

