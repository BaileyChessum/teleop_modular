# Used to describe the status of input sources, commands, or other plugin types

string name    # The name of the instance of the plugin
string type    # The pluginlib type name
bool ok        # Whether the plugin was loaded ok
string status  # Any additional information on the status of the plugin. Populated with error message on failure
