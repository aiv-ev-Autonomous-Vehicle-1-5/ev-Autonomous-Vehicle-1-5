# track_msgs/msg/LaneBoundary.msg
# Single lane boundary as a sequence of points

std_msgs/Header header
geometry_msgs/Point[] points    # Ordered boundary points

uint8 LEFT  = 0
uint8 RIGHT = 1
uint8 side                      # LEFT or RIGHT

float32 confidence              # Detection confidence (0.0 to 1.0)
