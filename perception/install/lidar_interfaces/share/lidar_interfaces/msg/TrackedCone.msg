std_msgs/Header header
uint32 track_id                   # Unique Tracking ID
geometry_msgs/Point position      # Filtered position (Kalman Filter)
geometry_msgs/Vector3 velocity    # Estimated velocity (vx, vy, vz)
geometry_msgs/Vector3 dimensions  # (width, depth, height)
float32 confidence
int32 label
