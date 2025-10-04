# Combines Inputs and InvokedEvents to make sure that values are synced when received.
# This is useful for infrequent inputs. Prefer two topics, for InputNames and CombinedInputValues for anything that
# frequently publishes a constant set of inputs.
#
# See the header comments for InputValues and InputNames!

teleop_msgs/InputNames names
teleop_msgs/InputValues values
teleop_msgs/InvokedEvents events
