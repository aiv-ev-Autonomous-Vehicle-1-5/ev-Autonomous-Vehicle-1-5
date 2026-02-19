std_msgs/Header header
geometry_msgs/Point position      # Centroid (x, y, z)
geometry_msgs/Vector3 dimensions  # (width, depth, height)
float32 confidence           # Detection confidence (0.0 to 1.0)
int32 label                  # Cluster label or class ID
