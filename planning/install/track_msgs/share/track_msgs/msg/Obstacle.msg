# track_msgs/msg/Obstacle.msg
# Single obstacle (static or dynamic)

geometry_msgs/Point position        # Centroid (x, y, z)
geometry_msgs/Vector3 dimensions    # (width, depth, height)
geometry_msgs/Vector3 velocity      # Estimated velocity (vx, vy, vz) for dynamic

uint8 TYPE_STATIC  = 0
uint8 TYPE_DYNAMIC = 1
uint8 type
