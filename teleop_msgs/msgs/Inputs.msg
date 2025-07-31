# Reports the state of a teleop_node's axes and buttons.
#
# Each input (buttons/axes) is uniquely identified by its name
# The header specifies the time at which the input values were received from the triggering input source.
# All inputs in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, two for each input type (buttons/axes)
# The goal is to make each of the fields optional. When e.g. your inputs have no input
# associated with them, you can leave the value array empty.
#
# All arrays for an input type in the message should have the same size, or be empty.
# This is the only way to uniquely associate the input name with the correct values.

Header header           # timestamp in the header is the time the data is received from the triggering input source

# Represent true/false values
string[] button_names
byte[] buttons          # the buttons values from a teleop_node

# Represent number values
string[] axis_names
float32[] axes          # the axis values from a teleop_node


