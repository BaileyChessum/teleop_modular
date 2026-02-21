# Current state of all control modes.

std_msgs/Header header    # timestamp in the header is the time of the last change

string[] names            # List of control modes.
bool[] active             # Whether each of those control modes is active.
