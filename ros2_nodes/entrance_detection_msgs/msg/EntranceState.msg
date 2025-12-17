# EntranceState.msg
# Current state and decision for entrance navigation

std_msgs/Header header

# State machine state
string state  # NORMAL, NIEDRIG, LIEGEND, BLOCKIERT, ANALYSE

# Required robot configuration
float32 required_height     # Required body height in meters
float32 safety_clearance    # Safety clearance above obstacle in meters

# Action flag
bool action_required  # True if robot must change posture

# Additional information
string description  # Human-readable state description
