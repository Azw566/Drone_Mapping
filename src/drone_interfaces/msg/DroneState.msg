# DroneState.msg
#
# Published by each drone at ~2Hz. Gives the coordinator a snapshot
# of what the drone is doing and where it is.
# This is the coordinator's "eyes" — without it, task allocation is blind.

# Which drone this is ("d1", "d2", etc.)
string drone_id

# Current 6DoF pose in the drone's own map frame.
# Comes from LIO-SAM odometry output.
geometry_msgs/PoseStamped current_pose

# Where the drone is currently flying toward (frontier centroid).
# A zero point means "no active goal" (idle).
geometry_msgs/Point current_goal

# What the drone is doing right now.
# Values: "exploring", "returning_home", "idle", "avoiding_collision", "landing"
# The coordinator won't reassign a drone that's "avoiding_collision".
string status

# Simulated battery level (0.0–100.0).
# In simulation this can be a simple timer-based drain.
# When it drops below a threshold, the drone should return to base.
float32 battery_percent
