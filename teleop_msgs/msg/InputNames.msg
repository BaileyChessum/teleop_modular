# Reports the names of a teleop_node's axes and buttons, usually to be paired with a complementary topic to publish
# InputValues.msg messages.
#
# Each input (buttons/axes) is uniquely identified by its name
#
# This message consists of a multiple arrays, one for each input type (buttons/axes)
# The goal is to be able to have topics that act like publishing Inputs.msg messages, but without the overhead of
# frequently publishing every name for every input
#
# Instead, this message can be used to provide the names of the inputs in a InputValues.msg message once,
# The arrays in InputValues.msg messages must then maintain the name number of elements in its arrays.
#
# All arrays for an input type in the InputValues.msg message and InputNames.msg message should have the same size, or
# be empty. This is the only way to uniquely associate the input name with the correct values.

# The name of the topic publishing the associated InputValues.msg.
# By convention, if left blank, this should be assumed as the name of the topic publishing this message,
# appended with "/values".
string value_topic

# Represent true/false values
string[] button_names

# Represent number values
string[] axis_names
