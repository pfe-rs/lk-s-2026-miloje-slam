# A single 360-degree lidar sweep, in the lidar's native units.
# angles[i] corresponds to distances[i] for every index i.

std_msgs/Header header

float32[] distances   # millimeters
float32[] angles      # degrees, 0-360

