# LidarGap.msg
# Detected gap in LiDAR scan (potential entrance)

std_msgs/Header header

# Gap properties
float32 width           # Width of gap in meters
float32 distance        # Distance to gap center in meters
float32 angle           # Angle in degrees (0 = forward)

# Gap location (2D Cartesian)
geometry_msgs/Point center   # (x, y, 0) in LiDAR frame

# Confidence
float32 quality         # Quality score 0.0 - 1.0
bool in_camera_fov      # True if gap is within camera field of view
