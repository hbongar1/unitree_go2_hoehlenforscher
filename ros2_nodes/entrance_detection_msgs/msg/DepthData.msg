# DepthData.msg
# Raw RealSense depth camera data

std_msgs/Header header

# Depth frame (640x480, 16-bit unsigned integers, z16 format)
sensor_msgs/Image depth_image

# Color frame (640x480, BGR8 format)
sensor_msgs/Image color_image

# Depth scale factor (converts depth units to meters)
# Typical value: ~0.001 (1 unit = 1mm)
float32 depth_scale

# Camera intrinsics for 3D projection
sensor_msgs/CameraInfo camera_info
