# MotorStatus.msg
# Feedback from motor controller (Unitree GO2)

std_msgs/Header header

# Current robot state
float32 current_body_height  # Current body height in meters
string posture_mode          # STAND, CROUCH, PRONE

# Motion status
bool in_motion    # True if currently executing height change
bool ready        # True if ready to accept new commands

# Error handling
string error_message  # Empty if OK, otherwise contains error description
