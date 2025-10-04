# Combines InputValues and InvokedEvents to make sure that values are synced when received, by say a python node
# Prefer using this message type in combination with a InputNames topic if you don't know what message type to choose.
#
# See the header comments for InputValues and InputNames!

teleop_msgs/InputValues values
teleop_msgs/InvokedEvents events
