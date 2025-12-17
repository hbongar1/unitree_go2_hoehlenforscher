# EntranceEvent.msg
# Detected entrance with 3D geometry information

std_msgs/Header header

# 3D position of entrance center in camera frame (meters)
geometry_msgs/Point position_3d

# 2D position of entrance center in image (pixels)
geometry_msgs/Point position_2d

# Entrance dimensions
float32 width      # Width in meters
float32 height     # Height in meters
float32 distance   # Distance from camera in meters

# Detection quality
float32 confidence  # Confidence score 0.0 - 1.0

# Passability assessment
bool is_passable   # True if entrance is wide and tall enough for robot
