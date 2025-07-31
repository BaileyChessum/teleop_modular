# Reports the values of a teleop_node's axes and buttons, usually to be paired with a complementary topic to publish
# InputNames.msg messages that associate each input value array with the name of the input.
#
# Each input (buttons/axes) is uniquely identified by its name
# The header specifies the time at which the input values were received from the triggering input source.
# All inputs in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each input type (buttons/axes)
# The goal is to be able to have topics that act like publishing Inputs.msg messages, but without the overhead of
# frequently publishing every name for every input
#
# The InputNames.msg message can be used to provide the names of the inputs in a InputValues.msg message once,
# The arrays in InputValues.msg messages must then maintain the name number of elements in its arrays.
#
# All arrays for an input type in the InputValues.msg message and InputNames.msg message should have the same size, or
# be empty. This is the only way to uniquely associate the input name with the correct values.

std_msgs/Header header           # timestamp in the header is the time the data is received from the triggering input source

# Represent true/false values
byte[] buttons          # the buttons values from a teleop_node

# Represent number values
float32[] axes          # the axis values from a teleop_node


